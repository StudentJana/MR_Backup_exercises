#include <memory>
#include <iostream>
#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "mr_pf/particle_filter_visualization.hpp"

using namespace mr;
using namespace tuw;
using namespace std;

ParticleFilterVisualization::ParticleFilterVisualization()
    : ParticleFilter()
{
}

bool ParticleFilterVisualization::init(ParticleFilterParameterPtr config)
{
    figure_ = make_shared<tuw::Figure>(config->name);
    figure_->init(config->map_cols, config->map_rows,
                  config->map_min_x, config->map_max_x,
                  config->map_min_y, config->map_max_y,
                  config->map_rotation + M_PI,
                  config->map_grid_x, config->map_grid_y, config->map_file);

    cv::namedWindow(figure_->title(), cv::WINDOW_GUI_NORMAL);
    return ParticleFilter::init(config);
}

void ParticleFilterVisualization::update(const tuw::Command2DConstPtr u, double dt)
{

    if(param_->sensor_model == ParticleFilterParameter::MODEL_SCAN){
        plot_likelihood_field();
    } 
    draw(1, dt);
    ParticleFilter::update(u, dt);
}

void ParticleFilterVisualization::callback_mouse(int event, int x, int y, int, void *param)
{
    ParticleFilterVisualization *node = reinterpret_cast<ParticleFilterVisualization *>(param);
    node->callback_mouse(event, x, y);
}

void ParticleFilterVisualization::callback_mouse(int event, int x, int y)
{

    /**
     * @ToDo Sensor & Motion Model
     * Reset the filter with a BUTTON to a defined pose
     * the position should be defined by the click (down) and the orienation by the button release (up)
     **/
    if (param_->level > ParticleFilterParameter::PLOT_LASER_MEASUREMENT)
    {
    }
    else
    {
        /// @node your code
        (void)x; /// to silence a warning about unused variables
        (void)y; /// to silence a warning about unused variables
    }
    if (event == cv::EVENT_RBUTTONUP)
    {
        reset_ = Reset::GROUND_TRUTH;
    }
    if (event == cv::EVENT_MBUTTONUP)
    {
        static int reset_last = Reset::GRID;
        if(reset_last == Reset::UNIFORM){
            reset_ = Reset::GRID;
        } else {
            reset_ = Reset::UNIFORM;
        }
        reset_last = reset_;
    }
}

void ParticleFilterVisualization::draw(int delay, double dt)
{
    figure_->clear();
    this->plot_laser_measurement_ground_truth();
    if(param_->sensor_model == ParticleFilterParameter::MODEL_BEAM && param_->draw_z_exp){
            plot_expected_measurments();
    } 
    this->plot_samples();
    char txt[0xFF];
    sprintf(txt, "duration = %3.2f, nr of samples %zu", dt, samples_.size());
    cv::putText(figure_->view(), txt, cv::Point(10, figure_->height() - 15), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(255, 255, 255), 5, cv::LINE_AA);
    cv::putText(figure_->view(), txt, cv::Point(10, figure_->height() - 15), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(128, 128, 128), 1, cv::LINE_AA);
    int spacing = 8;
    sprintf(txt, "%05zu us update", processing_time_update_.count());
    cv::putText(figure_->view(), txt, cv::Point(10, spacing * 1), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(128, 128, 128), 1, cv::LINE_AA);
    sprintf(txt, "%05zu us resample", processing_time_resample_.count());
    cv::putText(figure_->view(), txt, cv::Point(10, spacing * 2), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(128, 128, 128), 1, cv::LINE_AA);
    sprintf(txt, "%05zu us compute weight", processing_time_compute_weights_.count());
    cv::putText(figure_->view(), txt, cv::Point(10, spacing * 3), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(128, 128, 128), 1, cv::LINE_AA);

    cv::namedWindow(figure_->title(), cv::WINDOW_NORMAL);
    cv::resizeWindow(figure_->title(), figure_->view().cols, figure_->view().rows);
    cv::imshow(figure_->title(), figure_->view());
    cv::setMouseCallback(figure_->title(), callback_mouse, this);
    cv::waitKey(delay);
}

void ParticleFilterVisualization::plot_laser_measurement_ground_truth()
{
    if (ground_truth_)
    {
        figure_->symbol(*ground_truth_, 0.2, Figure::cyan, 1);
        plot_laser_measurement(*ground_truth_, z_r_);
    }
}

void ParticleFilterVisualization::plot_laser_measurement(const Pose2D &pose_vehicle, const vector<Point2D> &z)
{
    /**
     * @ToDo Sensor Model
     * Plot the laser measurement around a given robot pose pose.
     * Hint:
     *  Figure::symbol(Point2D)
     *  tf_base_sensor_
     *  pose_vehicle
     **/
    if (param_->level > ParticleFilterParameter::PLOT_LASER_MEASUREMENT)
    {
    }
    else
    {
        /// @note Your code here

        // tuw::Tf2D T = ...
        // for (...)
        // {
        //     figure_->symbol(T * ...);
        // }

        // Dummy code below should be removed.
        (void)pose_vehicle; /// to silence a warning about unused variables
        (void)z;            /// to silence a warning about unused variables
    }
}

/**
 * @ToDo Beam-based Sensor Model
 **/
void ParticleFilterVisualization::plot_expected_measurments()
{
    /**
     * @ToDo Sensor Model
     * Draw the expected_measurments_ into the figure_->view()
     **/
    if (param_->level > ParticleFilterParameter::PLOT_Z_EXPECTED)
    {
    }
    else
    {
        /// @note your code here
        // for (...)
        // {
        //    for (...)
        //    {
        //      ...
        //    }
        // }
    }
}

/**
 * @ToDo Sensor Model
 **/
void ParticleFilterVisualization::plot_likelihood_field()
{
    /**
     * @ToDo Sensor Model
     * Draw the likelihood field onto the background
     * Hint:
     * * cv::minMaxLoc
     **/
    if (param_->level > ParticleFilterParameter::PLOT_LIKELIHOOD_FIELD)
    {
    }
    else
    {
        /// @note your code here

        // double max = ...

        // for (...)
        // {
        //    for (...)
        //    {
        //      cv::Vec3b &px = figure_->background().at<cv::Vec3b>(r, c);
        //      px[0] = ...;
        //    }
        // }
    }
}

/**
 * @ToDo Sensor Model
 *
 *
 **/
void ParticleFilterVisualization::plot_samples()
{
    /**
     * @ToDo Sensor Model
     * draw all particles with weights
     * Hints:
     * @see tuw::Figure::symbol()
     **/
    if (param_->level > ParticleFilterParameter::PLOT_SAMPLES)
    {
    }
    else
    {
        /// @note your code here

        // double scale = 255.0 / samples_weight_max_;
        // for(auto sample : samples_) {
        //      figure_->symbol
        // }
    }
}
