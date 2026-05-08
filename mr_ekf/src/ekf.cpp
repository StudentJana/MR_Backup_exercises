#include <memory>
#include <iostream>
#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "mr_ekf/ekf.hpp"

using namespace mr;
using namespace tuw;
using namespace std;

EKF::EKF()
{
}
bool EKF::init(EKFParameter *param)
{
    param_ = param;
    reset_ = Reset::GROUND_TRUTH;
    hspace_header_.init(param_->hough_space_pixel_alpha, param_->hough_space_pixel_rho,
                        -M_PI * 1.1, +M_PI * 1.1,
                        0, param_->hough_space_meter_rho,
                        M_PI);
    return false;
}

void EKF::set_ground_truth(const Pose2D &p)
{
    if (!ground_truth_)
        ground_truth_ = make_shared<Pose2D>(p);
    else
        *ground_truth_ = p;
    ground_truth_->recompute_cached_cos_sin();
}

void EKF::set_init_pose(const Pose2D &p)
{
    if (!init_pose_)
        init_pose_ = make_shared<Pose2D>(p);
    else
        *init_pose_ = p;
}

void EKF::set_map(const LineSegments2D &l)
{
    map_linesegments_ = l;
}

Pose2DPtr EKF::compute_estimated_pose()
{
    return estimated_pose_;
}

void EKF::prediction(const Command2DConstPtr u, double dt)
{
    reset();
    if (!estimated_pose_ || !u || fabs(dt) <= FLT_MIN)
        return;

    x = estimated_pose_->state_vector();
    if (param_->enable_prediction)
    {

        /**
         * @ToDo  pose prediction and covariance prediction
         * compute KalmanFilter::xp and KalmanFilter::Pp as predicted pose and Covariance
         * Computes the prediction step of the Kalman filter as in Thrun et. al.
         * In special cases: calculate limit for angular velocity to 0.
         **/
        (void) u;
        (void) dt;
        // pose prediction
        xp = x;
        // covariance prediction
        Pp = P;
    }
    else
    {
        xp = x;
        Pp = P;
    }
    if (!pose_predicted_)
        pose_predicted_ = make_shared<Pose2D>();
    pose_predicted_->set(xp);
}

void EKF::reset()
{
    switch (reset_)
    {
    case Reset::UNIFORM:
        estimated_pose_ = make_shared<Pose2D>();
        P = cv::Matx33d(param_->init_sigma_location * param_->init_sigma_location, 0, 0,
                        0, param_->init_sigma_location * param_->init_sigma_location, 0,
                        0, 0, param_->init_sigma_orientation * param_->init_sigma_orientation);
        reset_ = Reset::OFF;
        break;
    case Reset::GROUND_TRUTH:
        if (ground_truth_)
        {
            estimated_pose_ = make_shared<Pose2D>(*ground_truth_);
            P = cv::Matx33d(param_->init_sigma_location * param_->init_sigma_location, 0, 0,
                            0, param_->init_sigma_location * param_->init_sigma_location, 0,
                            0, 0, param_->init_sigma_orientation * param_->init_sigma_orientation);
            reset_ = Reset::OFF;
        }
        break;
    case Reset::INTI_POSE:
        if (init_pose_)
        {
            estimated_pose_ = make_shared<Pose2D>(*init_pose_);
            P = cv::Matx33d(param_->init_sigma_location * param_->init_sigma_location, 0, 0,
                            0, param_->init_sigma_location * param_->init_sigma_location, 0,
                            0, 0, param_->init_sigma_orientation * param_->init_sigma_orientation);
            reset_ = Reset::OFF;
        }
        break;
    default:
        break;
    }
}

void EKF::correction(StampedDataPtr<LineSegments2D> z)
{
    measurement_linesegments_ = z;
    if (!pose_predicted_)
        return;
    data_association();

    xc = pose_predicted_->state_vector();
    Pc = Pp;

    if (param_->enable_correction)
    {
        Q = cv::Matx<double, 2, 2>(param_->measurement_noise_alpha, 0, 0, param_->measurement_noise_rho);
        for (size_t idx_measurement = 0; (idx_measurement < measurement_match_.size()); idx_measurement++)
        {
            int idx_map = measurement_match_[idx_measurement];
            cv::Matx<double, 2, 3> H;             /// Check slides
            cv::Matx<double, 2, 1> v;             /// Measurement error between prediction (known data) and detection --> Siegwart;
            cv::Matx<double, 2, 2> Si;            /// Check slides
            cv::Matx<double, 3, 2> K;             /// Kalman gain
            cv::Matx<double, 3, 1> dx;            /// State change
            cv::Matx<double, 1, 1> d_mahalanobis; // just for debugging reasons, not needed;

            /**
             * @ToDo pose correction
             * Pose correction must update the KalmanFilter::xc and KalmanFilter::Pc which represents the corrected pose with covariance
             * have a look into Siegwart 2004 section 5.6.3.3 Case study: Kalman filter localization with line feature extraction
             * Siegwart correction implementation
             */
            (void) idx_map;
        }
    }
    if (!estimated_pose_)
        estimated_pose_ = make_shared<Pose2D>();
    estimated_pose_->set(xc);
    P = Pc;
}

void EKF::correction(StampedDataPtr<Points2D> z)
{
    measurement_laser_scan_ = z;
}

void EKF::data_association()
{
    measurement_match_.clear();
    if (!pose_predicted_)
        return;
    /**
     * @ToDo predicted linesegments
     * compute the measurement prediction (predicted_linesegments_)
     * The predicted_linesegments_ are map_linesegments_ in the robots pose prediction frame
     */


    if (!param_->enable_data_association)
        return;
    measurement_match_.resize(measurement_linesegments_->data.size(), -1);
    for (size_t i = 0; i < measurement_linesegments_->data.size(); i++)
    {
        Polar2D measurement = measurement_linesegments_->data[i].toPolar();
        float dMin = FLT_MAX;
        measurement_match_[i] = -1;
        for (size_t j = 0; j < predicted_linesegments_.size(); j++)
        {
            Polar2D prediction = predicted_linesegments_[j].toPolar();
            /**
             * @ToDo matching measurement with prediction
             * Find the best measurement prediction idx j and store it in measurement_match_[i].
             * Here the hough transform is used to match the lines as seen from the prediction (predicted_linesegments)
             * with the currently measured linesegments. Both are in the same coordinate space (prev step) and are transformed
             * into hough space (lines are points here and can be easily matched).
             * Rqt allows for defining a matching threshold (distance)
             * Implement the preselection critea mentioned in the slides.
             */
            (void) measurement;
            (void) dMin;
            (void) prediction;
        }
    }
}