#include <memory>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <numbers>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "mr_pf/particle_filter.hpp"

using namespace mr;
using namespace tuw;
using namespace std;

ParticleFilter::ParticleFilter()
    : generator_(rd_())
{
}
bool ParticleFilter::init(ParticleFilterParameterPtr param)
{
    param_ = param;

    map_header_.init(param_->map_cols, param_->map_rows,
                     param_->map_min_x, param_->map_max_x,
                     param_->map_min_y, param_->map_max_y,
                     param_->map_rotation + M_PI);

    if (map_header_.scale_x() != map_header_.scale_y())
    {
        std::cerr << "loadMap: non-symmetric scale!";
        return true;
    }
    cv::Mat image = cv::imread(param_->map_file, cv::IMREAD_GRAYSCALE);
    cv::resize(image, map_, cv::Size(param_->map_cols, param_->map_rows), cv::INTER_AREA);

    distance_field_pixel_.create(param_->map_cols, param_->map_rows);
    likelihood_field_.create(param_->map_cols, param_->map_rows);
    expected_measurments_.create(param_->map_cols, param_->map_rows);

    uniform_x_ = std::uniform_real_distribution<double>(param_->map_min_x, param_->map_max_x);
    uniform_y_ = std::uniform_real_distribution<double>(param_->map_min_y, param_->map_max_y);
    uniform_theta_ = std::uniform_real_distribution<double>(-M_PI, M_PI);
    uniform_ = std::uniform_real_distribution<double>(0., 1.);
    reset_ = Reset::UNIFORM;
    return false;
}


void ParticleFilter::sample_grid() {
    float angle_division = 16;
    double samples_per_angle = samples_.size() / angle_division;
    double A = (param_->map_max_x - param_->map_min_x) * (param_->map_max_y - param_->map_min_y);
    double samples_per_m2 = samples_per_angle / A;
    double d = 1.0 / sqrt(samples_per_m2);
    size_t i = 0;
    for (double x = param_->map_min_x + d / 2.; x < param_->map_max_x; x += d) {
        for (double y = param_->map_min_y + d / 2.; y < param_->map_max_y; y += d) {
            for (double theta = -M_PI; theta < M_PI; theta += (2. * M_PI) / angle_division) {
                if(i >= samples_.size()) return;
                SamplePose2DPtr &des = samples_[i++];
                des->x() = x;
                des->y() = y;
                des->theta() = theta;
                des->recompute_cached_cos_sin();
            }
        }
    }
}

SamplePose2DPtr &ParticleFilter::sample_uniform(SamplePose2DPtr &des)
{
    des->x() = uniform_x_(generator_);
    des->y() = uniform_y_(generator_);
    des->theta() = uniform_theta_(generator_);
    des->recompute_cached_cos_sin();
    return des;
}
SamplePose2DPtr &ParticleFilter::sample_normal(SamplePose2DPtr &des, tuw::Pose2D &src, double sigma_position, double sigma_orientation)
{
    des->x() = src.x() + normal_distribution_(generator_) * sigma_position;
    des->y() = src.y() + normal_distribution_(generator_) * sigma_position;
    des->theta() = src.theta() + normal_distribution_(generator_) * sigma_orientation;
    des->recompute_cached_cos_sin();
    return des;
}
void ParticleFilter::set_ground_truth(const Pose2D &p)
{
    if (!ground_truth_)
        ground_truth_ = make_shared<Pose2D>(p);
    else
        *ground_truth_ = p;
}
void ParticleFilter::set_init_pose(const Pose2D &p)
{
    if (!init_pose_)
        init_pose_ = make_shared<Pose2D>(p);
    else
        *init_pose_ = p;
}

void ParticleFilter::reset_samples()
{

    switch (reset_)
    {
    case Reset::UNIFORM:
        for (auto &s : samples_)
        {
            sample_uniform(s);
        }
        break;
    case Reset::GRID:
        sample_grid();
        break;
    case Reset::GROUND_TRUTH:
        if (ground_truth_)
        {
            for (auto &s : samples_)
            {
                sample_normal(s, *ground_truth_, param_->resample_noise_position, param_->resample_noise_orientation);
            }
        }
        break;
    case Reset::INTI_POSE:
        if (init_pose_)
        {
            for (auto &s : samples_)
            {
                sample_normal(s, *init_pose_, param_->resample_noise_position, param_->resample_noise_orientation);
            }
        }
        break;
    default:
        break;
    }
    if (!param_->continues_reset)
        reset_ = Reset::OFF;
}
void ParticleFilter::generate_samples()
{
    while (samples_.size() < param_->nr_of_samples)
    {
        SamplePose2DPtr s = std::make_shared<SamplePose2D>();
        samples_.push_back(sample_uniform(s));
    }
    while (samples_.size() > param_->nr_of_samples)
    {
        samples_.pop_back();
    }
}

void ParticleFilter::compute_weights(const vector<std::pair<tuw::Point2D, tuw::Polar2D>> &z_s, const tuw::Pose2D &pose_laser, double range_max)
{
    if(z_s.empty()) return;
    std::chrono::steady_clock::time_point time_weighting_last_ = std::chrono::steady_clock::now();


    /// reduce or increase number of samples
    generate_samples();

    tf_base_sensor_ = pose_laser.tf(); /// make a copy of the laser frame

    /// create a copy of the current laser measurement
    z_r_.resize(z_s.size());
    vector<std::pair<tuw::Point2D, tuw::Polar2D>>::const_iterator src;
    vector<tuw::Point2D>::iterator des;
    for (src = z_s.begin(), des = z_r_.begin(); (src < z_s.end()) && (des < z_r_.end()); src++, des++)
        (*des) = (*src).first;

    std::vector<size_t> used_beams(std::floor(std::clamp(param_->rate_of_beams_used, 0., 1.) * (z_s.size() - 1))); /// vector of beam indexes used

    /**
     * @ToDo Sensor Model
     * Select beams either randomly or equally distributed with a if-else statement.
     **/
    if (param_->level >= ParticleFilterParameter::COMPUTE_BEAMS_USED)
    {
    }
    else
    {
        /**
         * @note your code here
         **/
        int step_size = z_s.size() / used_beams.size();
        for (size_t i = 0; i < used_beams.size(); i++)
        {
            size_t idx = i * step_size + step_size / 2.;
            used_beams[i] = idx;
        }
    }
    double weight_sum = 0;


    /**
     * @todo Sensor Model - Scan
     * Compute the particle weight using the scan-based sensor model.
     * 0. Before implementing this TODO, ensure the following are ready:
     * * ParticleFilter::likelihood_field_
     * 1. used_beams defines the indices of the laser beams to be processed.
     * 2. Use the likelihood_field_ lookup for each laser measurement.
     * 3. Compute particle weights according to the textbook and lecture slides.
     */
    auto weight_sample_scan_model = [&](SamplePtr<Pose2D> &s)
    {

        cv::Matx33d M = map_header_.Mw2m() * s->tf() * pose_laser.tf();
        double q = 1;
        for (size_t i : used_beams)
        {
            const std::pair<tuw::Point2D, tuw::Polar2D> &beam = z_s[i];
            if (param_->level > ParticleFilterParameter::COMPUTE_SAMPLES_WEIGHT)
            {
            }
            else
            {
                /**
                 * @note your code here for the scan-based sensor model
                 * Do not forget to check if the laser beam is inside the map
                 **/
                (void)M;    /// to silence a warning about unused variables
                (void)beam; /// to silence a warning about unused variables

                // double qr = param_->z_rand / range_max;
                // Point2D p = M * beam.first;

                // qh = param_->z_hit * ...

                // q *= ...
            }
        }
        s->weight() = q;
        weight_sum += s->weight();
    };


    /**
     * @todo Sensor Model - Beam
     * Compute the particle weight using the beam-based sensor model.
     * 0. Before implementing this TODO, ensure the following are ready:
     * * ParticleFilter::compute_expected_measurement
     * * ParticleFilter::pseudo_density_fnc_ 
     * 1. used_beams defines the indices of the laser beams to be processed.
     * 2. Use the pseudo_density_fnc_ lookup for each laser measurement.
     * 3. Compute particle weights according to the textbook and lecture slides.
     */
    auto weight_sample_beam_model = [&](SamplePtr<Pose2D> &s)
    {

        tuw::Tf2D M = map_header_.Mw2m() * s->tf() * pose_laser.tf();

        double q = 1;
        for (size_t i : used_beams)
        {
            const std::pair<tuw::Point2D, tuw::Polar2D> &beam = z_s[i];
            double z_exp = compute_expected_measurment(beam.second, M, range_max);

            if (param_->level > ParticleFilterParameter::COMPUTE_SAMPLES_WEIGHT)
            {
            }
            else
            {
                /**
                 * @note your code here for the beam-based sensor model
                 * Do not forget to check if the laser beam is inside the map
                 **/
                (void)beam; /// to silence a warning about unused variables
                (void)z_exp; /// to silence a warning about unused variables

                // double p_zi = pseudo_density_fnc_(r,c);
                // q *= p_zi
            }
        }
        s->weight() = q;
        weight_sum += s->weight();
    };

    if(param_->sensor_model == ParticleFilterParameter::MODEL_SCAN){
        /// compute or update likelihood field
        compute_likelihood_field();
        std::for_each(samples_.begin(), samples_.end(), weight_sample_scan_model);
    } else if(param_->sensor_model == ParticleFilterParameter::MODEL_BEAM){
        /// precompute pseudo-density function
        compute_pseudo_density_lookup(range_max);
        expected_measurments_.setTo(0);
        std::for_each(samples_.begin(), samples_.end(), weight_sample_beam_model);
    }
    std::sort(samples_.begin(), samples_.end(), Sample<Pose2D>::greater);

    samples_weight_max_ = 0;
    for (size_t i = 0; i < samples_.size(); i++)
    {
        SamplePtr<Pose2D> &s = samples_[i];
        s->weight() /= weight_sum;
        s->idx() = i;
        if (s->weight() > samples_weight_max_)
            samples_weight_max_ = s->weight();
        // std::cout << s->idx() << ": " << s->weight() << std::endl;
    }
    processing_time_compute_weights_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_weighting_last_);
}


double ParticleFilter::compute_expected_measurment(const tuw::Polar2D &z_i, const tuw::Tf2D &M, double range_max){

    double z_exp = range_max;
    Point2D ps_zero(0.,0.,1.);                                   /// sensor mount in sensor system [meter]
    cv::Point pm_zero = (M * tuw::Point2D(0.,0.,1.)).cv();       /// sensor mount in map system [pixel]
    /**
     * @todo Sensor Model compute expected measurment
     * Compute the expected measurement:
     * 1. Compute sensor position pm_zero in map coordinates [pixels].
     * 2. Compute max reading position pm_max in map coordinates [pixels].
     * 3. Check for obstacles along the line from pm_zero to pm_max. 
     *    Hint: Use cv::LineIterator, 
     * 4. Use map_header_.scale_x() and map_header_.scale_y() to compute z_exp.
     * 5. For debugging, draw the pixels up to the obstacle or range_max into expected_measurements_.
     */        
    if (param_->level > ParticleFilterParameter::COMPUTE_Z_EXPECTED){    
    } else {
        /**
         * @note your code here
         * The following lines should give you some hints
         **/

        cv::line(expected_measurments_, pm_zero, (M * Point2D(1., 0., 1.)).cv(), cv::Scalar(0xEF));  /// remove
        (void)z_i; /// to silence a warning about unused variables
        /*
        ...
        cv::LineIterator it(map_, pm_zero, pm_max, 8);
        for (int i = 0; i < it.count; ++it, ++i){    
            uint8_t v = **it; 
            pm =  it.pos();      
            if(v == 0) {
               
                ... 
                z_exp = 
            } else {
                ...
                expected_measurments_(pm) = 0xFF;         /// Debug drawing
            }
        } 
        **/
    }
    return z_exp;
}

void ParticleFilter::compute_pseudo_density_lookup(double range_max){
    static ParticleFilterParameter::Level level = ParticleFilterParameter::CHECK_MAP_FILE_EXISTS;
    static double scale = -1;
    static double lambada_short = -1;
    static double sigma_hit = -1;
    static double z_hit = -1;
    static double z_short = -1;
    static double z_max = -1;
    static double z_rand = -1;
    static double gnuplot_z_exp = -1;
    
    /// Check to if the computation is needed (changes on the parameter)
    if((level == param_->level) && 
        (scale == param_->pseudo_density_scale) && 
        (lambada_short == param_->lambada_short) && 
        (sigma_hit == param_->sigma_hit) && 
        (z_hit == param_->z_hit) && 
        (z_short == param_->z_short) && 
        (z_max == param_->z_max) && 
        (z_rand == param_->z_rand) && 
        (gnuplot_z_exp == param_->gnuplot_z_exp)){
        return;
    }
    level = param_->level;
    scale = param_->pseudo_density_scale;
    sigma_hit = param_->sigma_hit;
    lambada_short = param_->lambada_short;
    z_hit = param_->z_hit;
    z_short = param_->z_short;
    z_max = param_->z_max;
    z_rand = param_->z_rand;
    gnuplot_z_exp = param_->gnuplot_z_exp;
    int size = range_max * param_->pseudo_density_scale;
    pseudo_density_fnc_.create(size, size);

    // Lambda for normal PDF
    auto normal_pdf = [](double x, double mean, double stddev) {
        double z = (x - mean) / stddev;
        return std::exp(-0.5 * z * z) / (stddev * std::sqrt(2.0 * std::numbers::pi));
    };

    std::shared_ptr<std::ofstream> file;
    if(gnuplot_z_exp > 0){
        file = std::make_shared<std::ofstream>("/tmp/pseudo-density.txt");
        if (!file->is_open()) {
            std::cout << "could not open pseudo-density.txt" << std::endl;
            return;
        };
    }
    /**
     * @todo Sensor Model - Beam
     * Compute the pseudo density lookup:
     * 1. Compute pseudo_density_fnc_
     * 2. Compute a gnuplot debug output for gnuplot_z_exp
     */   
    if (param_->level > ParticleFilterParameter::COMPUTE_PSEUDO_DENSITY_LOOKUP){    
    } else {
        /**
         * @note your code here
         * The following lines should give you some hints
         **/
        (void)normal_pdf; /// to silence a warning about unused variables

        /// double eta_z = ...
        for(int r = 0; r < pseudo_density_fnc_.rows; r++){
            double z_exp = r / scale;
            double z_exp_next = (r+1) / scale;
            // double eta_short = ...
            bool plot = file && z_exp <= gnuplot_z_exp && z_exp_next > gnuplot_z_exp;
            for(int c = 0; c < pseudo_density_fnc_.cols; c++){
                double p_hit = 0;
                double p_short = 0;
                double p_max = 0;
                double p_rand = 0;
                double p_pseudo_density = 0;
                double z = c / scale;

                // p_hit = ...
                // p_short = ...
                // p_max = ...
                // p_rand = ...
                // p_pseudo_density = ...

                if(plot){
                    *file << z << " " << p_hit << " " << p_short << " " << p_max << " " << p_rand  << " " << p_pseudo_density << std::endl;
                }
                pseudo_density_fnc_(r,c) = p_pseudo_density; 
            }
        }    

    }
    if(file) file->close();
}

void ParticleFilter::compute_likelihood_field()
{
    static double sigma = 0.;
    static ParticleFilterParameter::Level level = ParticleFilterParameter::CHECK_MAP_FILE_EXISTS;
    /// Check to compute the likelihood field only if needed (changes on the parameter)
    if ((sigma == param_->sigma_hit) && (level == param_->level) && (param_->sigma_hit <= 0))
        return;
    sigma = param_->sigma_hit;
    level = param_->level;

    // Lambda for normal PDF
    auto normal_pdf = [](double x, double mean, double stddev) {
        double z = (x - mean) / stddev;
        return std::exp(-0.5 * z * z) / (stddev * std::sqrt(2.0 * std::numbers::pi));
    };

    /**
     * @ToDo Sensor Model
     * using the cv::distanceTransform and the lambda normal_pdf
     * Here, the likelihood field is computed using the following steps.
     * - First the distance transform is called on the map which outputs the distance to the nearest pixels holding the value 0
     *   of all other pixels. Use the matrix distance_field_pixel_ to store the result.
     * - To transform from pixels to metric values, the field has to be divided by a scale.
     *   Use the matrix distance_field_ to store the result.
     * - The pdf can then be used on the metric distance field to obtain the likelihood values.
     *   Use the matrix likelihood_field_ to store the result.
     **/ 
    if (param_->level > ParticleFilterParameter::COMPUTE_LIKELIHOOD_FIELD)
    {
    }
    else
    {
        /// @note your code
        /// replace the following lines with your code
        (void)normal_pdf; /// to silence a warning about unused variables

        // cv::distanceTransform(...)
        // Scale from px to meters ...

        for (int r = 0; r < likelihood_field_.rows; r++)
        {
            for (int c = 0; c < likelihood_field_.cols; c++)
            {
                // boost::math::pdf(normal_distribution, ...)
                // likelihood_field_(r, c) =

                // Dummy code below, replace with your code
                float v = (float)c / (float)likelihood_field_.cols;
                likelihood_field_(r, c) = v;
            }
        }
    }
}

void ParticleFilter::update(const Command2DConstPtr u, double dt)
{

    reset_samples();
    if (!u || param_->disable_update)
        return;
    std::chrono::steady_clock::time_point time_update_start = std::chrono::steady_clock::now();

    for (SamplePose2DPtr s : samples_)
    {
        /**
         * @ToDo Motion model
         * implement the forward sample_motion_velocity algorithm and be aware that w can be zero!!
         * use the param_->alpha1 - param_->alpha6 as noise parameters
         * Executes the forward prediction step for each particle
         **/
        if (param_->level > ParticleFilterParameter::MOTION_UPDATE)
        {
        }
        else
        {
            /// @node your code
            (void)dt; /// to silence a warning about unused variables
        }
    }
    processing_time_update_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_update_start);
}
void ParticleFilter::resample(double dt)
{
    if (samples_.empty())
        return;
    std::chrono::steady_clock::time_point time_resample_start = std::chrono::steady_clock::now();
    std::uniform_int_distribution<size_t> uniform_idx_des(0, samples_.size() - 1);

    std::vector<SamplePose2DPtr> samples_old = samples_;                            /// old sample set
    size_t M = samples_.size() * std::clamp<double>(param_->resample_rate, 0., 1.); /// number of cloned samples
    /**
     * @ToDo Resample
     * implement a resample wheel
     * Implementation of two different resampling steps is required for the exercise
     */
    if (param_->level > ParticleFilterParameter::RESAMPLING)
    {
    }
    else
    {
        /// @node your code
        (void)dt; /// to silence a warning about unused variables
        (void)M;  /// to silence a warning about unused variables
        if (param_->resample_strategy == ParticleFilterParameter::KEEP_BEST)
        {
            /// simple resample strategy witch keeps the best n-M best
        }
        else if (param_->resample_strategy == ParticleFilterParameter::LOW_VARIANCE)
        {
            /// low variance strategy
        }
    }
    processing_time_resample_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_resample_start);
}
tuw::Pose2DPtr ParticleFilter::compute_estimated_pose()
{
    return estimated_pose_;
}
