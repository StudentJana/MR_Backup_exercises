#include <memory>
#include <iostream>
#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "mr_ekf/ekf_visualization.hpp"

using namespace mr;
using namespace tuw;
using namespace std;

EKFVisualization::EKFVisualization()
    : EKF()
{
}

bool EKFVisualization::init(EKFParameter *param)
{
    param_ = (EKFVizParameter *)param;
    return EKF::init(param);
}

void EKFVisualization::prediction(const tuw::Command2DConstPtr u, double dt)
{
    this->draw();
    EKF::prediction(u, dt);
}

void EKFVisualization::callback_mouse(int event, int x, int y, int, void *param)
{
    EKFVisualization *node = reinterpret_cast<EKFVisualization *>(param);
    node->callback_mouse(event, x, y);
}

void EKFVisualization::callback_mouse(int event, int x, int y)
{

    static Point2D point;
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        point.set(figure_map_->m2w(Point2D(x, y)));
    }
    if (event == cv::EVENT_LBUTTONUP)
    {
        /**
         * @ToDo create a init pose from mouse click
         * similar to the particle filter
         **/
        reset_ = Reset::INTI_POSE;
    }
    if (event == cv::EVENT_RBUTTONUP)
    {
        reset_ = Reset::GROUND_TRUTH;
    }
    if (event == cv::EVENT_MBUTTONUP)
    {
        reset_ = Reset::UNIFORM;
    }
}

void EKFVisualization::draw(int delay)
{
    if (!figure_map_)
    {
        figure_map_ = make_shared<tuw::Figure>(param_->name);
        figure_map_->setLabel("x=%4.1f", "y=%4.1f");
        figure_map_->init(param_->map_cols, param_->map_rows,
                          param_->map_min_x, param_->map_max_x,
                          param_->map_min_y, param_->map_max_y,
                          param_->map_rotation + M_PI,
                          param_->map_grid_x, param_->map_grid_y, param_->map_file);
        cv::namedWindow(figure_map_->title(), cv::WINDOW_GUI_NORMAL);
        cv::moveWindow(figure_map_->title(), 20, 20);
        cv::resizeWindow(figure_map_->title(), 650, 650);
    }
    figure_map_->clear();
    if (ground_truth_)
    {
        figure_map_->symbol(*ground_truth_, 0.2, Figure::cyan, 1);
    }
    if (estimated_pose_)
    {
        figure_map_->symbol(*estimated_pose_, 0.2, Figure::green, 1);
        draw_measurement(*estimated_pose_);
        draw_covariance();
    }

    draw_hspace();
    /**
     * @ToDo write your name into the view
     **/
    cv::imshow(figure_map_->title(), figure_map_->view());
    cv::setMouseCallback(figure_map_->title(), callback_mouse, this);
    cv::waitKey(delay);
}

void EKFVisualization::draw_covariance()
{
    /**
     * @ToDo visualize the pose covariance with an ellipse which covers 95 %
     * Compute and plot the pose covariance in x and y direction
     * take the pose covariance P and create a 2x2 matrix out of the x,y components
     * transform the matrix into the plot E = Mw2m*P(0:1,0:1)*Mw2m'
     * use the opencv to compute eigenvalues and eigen vectors to compute the size and orientation of the ellipse
     * After the prediction, the ellipse of the covariance should be plotted here
     * Eigenvalues are used to visualize the ellipse with opencv ellipse
     * atan2 is needed to always return the angle from the x-axis and eliminate ambiguity (as opposed to atan)
     **/
    (void) x;
    (void) P;
    cv::Matx<double, 2, 2> E ( 1, 0, 0, 1 );  /// must be changed
    cv::Mat_<double> eigval, eigvec;
    cv::eigen ( E, eigval, eigvec );
    cv::RotatedRect ellipse ( cv::Point(100,50),cv::Size ( 49,29 ), 93 ); /// must be changed
    cv::ellipse ( figure_map_->view(),ellipse, Figure::magenta, 1, cv::LINE_AA );
}
void EKFVisualization::draw_measurement(const Pose2D &pose_vehicle)
{
    cv::Scalar color = Figure::orange;
    /**
     * @ToDo drawing of the known map_linesegments_
     * visualize the line map (future map) stored in map_linesegments_
     * draw a corresponding number to the center of each linesegment
     **/
    (void) color;

    if (measurement_laser_scan_)
    {
        cv::Scalar color = Figure::orange;
        /**
         * @ToDo visualize the laser scan stored in measurement_laser_scan_
         **/
        (void) pose_vehicle;
        (void) color;
    }
    if (measurement_linesegments_)
    {

        cv::Scalar color_measurement = Figure::red;
        cv::Scalar color_match = Figure::blue;
        /**
         * @ToDo visualize the line segments stored in measurement_linesegments_
         * and the line matching stored in measurement_match_ with the line map.
         * We suggest to implement first the line drawing and after
         * you implemented EKF::data_association to enhance the drawing with
         * by visualization of the matching.
         **/
        (void) pose_vehicle;
        (void) color_measurement;
        (void) color_match;
    }
}

void EKFVisualization::draw_hspace()
{

    if (!figure_hspace_)
    {
        /// Init hspace figure
        figure_hspace_ = std::make_shared<Figure>(param_->name + std::string(" Hough Space"));
        figure_hspace_->setLabel("alpha=%4.2f", "rho=%4.2f");
        figure_hspace_->init(param_->hough_space_pixel_alpha, param_->hough_space_pixel_rho,
                             -M_PI * 1.1, +M_PI * 1.1,
                             0, param_->hough_space_meter_rho,
                             M_PI,
                             1, M_PI / 4);
        cv::namedWindow(figure_hspace_->title(), cv::WINDOW_GUI_NORMAL | cv::WINDOW_NORMAL);
        cv::moveWindow(figure_hspace_->title(), 720, 20);
        cv::resizeWindow(figure_hspace_->title(), 650, 650);
    }

    figure_hspace_->clear();
    cv::Rect rectSpace(0, 0, figure_hspace_->view().cols, figure_hspace_->view().rows);
    if (measurement_laser_scan_)
    {
        for (unsigned int i = 0; i < measurement_laser_scan_->data.size(); i++)
        {
            Point2D p0 = measurement_laser_scan_->data[i];
            for (double alpha = figure_hspace_->min_x(); alpha < figure_hspace_->max_x(); alpha += 1.0 / figure_hspace_->scale_x())
            {
                /**
                 * @ToDo laser beams in hough space
                 * draw a wave with angle = [-pi...pi], r = x*cos(angle) + y *sin(angle) for every laser point [x,y].
                 * The function Line2D::toPolar() can be used to transform a line into polar coordinates
                 * Plot the lines (measurements) in hough coordinates.
                 * Each laser scan point that corresponds to a line is transformed into hough coordinates here.
                 * The intersection of all of them yields the resulting value for the hough representation of each line
                 **/
                (void) p0;
            }
        }
    }
    cv::Scalar color;
    if (measurement_linesegments_)
    {
        /**
         * @ToDo measurement in hough space
         * Draw the measurement (measurement_linesegments_) in the hough space as a circle or dot.
         * You should also write the index of each line segment next to it for debugging.
         */
        color = Figure::red;
    }
    /**
     * @ToDo prediction measurement in hough space
     * Draw the measurement prediction (predicted_linesegments_) in the hough space as a circle or dot.
     * You should also write the index of each line segment next to it for debugging.
     */
    color = Figure::orange;
    (void) color;


    cv::imshow(figure_hspace_->title(), figure_hspace_->view());
}
