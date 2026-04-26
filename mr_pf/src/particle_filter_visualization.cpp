#include <memory>
#include <iostream>
#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "mr_pf/particle_filter_visualization.hpp"
#include <boost/range/adaptor/reversed.hpp>

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
        static Point2D down_coordinates;
        //static cv::Point down_coordinates;
        static bool down_pressed=false;

        if (event==cv::EVENT_LBUTTONDOWN){
            down_pressed=true;
            down_coordinates=Point2D(x,y);
            //down_coordinates=cv::Point(x, y);
        }
        else if (event== cv::EVENT_LBUTTONUP && down_pressed){
            down_pressed=false;

            //cv::Point up_coordinates(x, y);
            Point2D up_coordinates(x,y);
            Point2D down_coordinates_world= figure_->m2w(down_coordinates);
            Point2D up_coordinates_world= figure_->m2w(up_coordinates);


            float theta=atan2(up_coordinates_world.get_y()-down_coordinates_world.get_y(), up_coordinates_world.get_x()-down_coordinates_world.get_x());
            
            Pose2D p(down_coordinates_world, theta);
            set_init_pose(p);

            reset_=Reset::INTI_POSE;
            this->reset_samples();
        }

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

        
        tuw::Tf2D T= pose_vehicle.tf() * tf_base_sensor_;
        for (size_t i = 0; i < z.size(); i++){
            figure_->symbol(T * z[i], 0.05, tuw::Figure::magenta, 2);
        }
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
        for (int r=0; r < expected_measurments_.rows; r++)
        {
           for (int c=0; c < expected_measurments_.cols; c++)
           {
             if (expected_measurments_(r,c)>0){
                cv::Vec3b &px = figure_->view().at<cv::Vec3b>(r, c);             
                    
                px[0] = 0;   
                px[1] = 0;   
                px[2] = 255; 
             }
           }
        }
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
        double max;
        cv::minMaxLoc(likelihood_field_, nullptr, &max);

        // std::cout << "DEBUG  Max probability in field: " << max << std::endl; //DEBUG

        for (int r = 0; r < likelihood_field_.rows; r++)
        {
            for (int c = 0; c < likelihood_field_.cols; c++)           
            {
                cv::Vec3b &px = figure_->background().at<cv::Vec3b>(r, c);             
                px[0] =max >0 ?255- (uchar)(likelihood_field_(r,c)/max*255):0;
            }
        }

        // //DEBUG
        // std::cout << "DEBUG - 5x5 Center of likelihood_field_:" << std::endl;
        // int center_r = likelihood_field_.rows / 2;
        // int center_c = likelihood_field_.cols / 2;

        // // Print a 5x5 square around the center pixel
        // for (int r = center_r - 2; r <= center_r + 2; r++) {
        //     for (int c = center_c - 2; c <= center_c + 2; c++) {
        //         std::cout << likelihood_field_.at<double>(r, c) << "  ";
        //     }
        //     std::cout << std::endl;
        // }
        //END DEBUG
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

        double scale = 255.0 / samples_weight_max_;
        for(auto sample : boost::adaptors::reverse(samples_)) {
            double px= sample->get_x();
            double py= sample->get_y();
            double value=sample->weight()*scale;
            cv::Scalar color(255-value, value, 0.0);
            figure_->symbol(Point2D(px,py), 0.1, color );
        }
    }
}
