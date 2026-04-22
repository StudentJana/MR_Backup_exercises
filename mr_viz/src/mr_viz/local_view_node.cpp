
#include <chrono>
#include <thread>
#include "mr_viz/local_view_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using std::placeholders::_1;

LocalViewNode::LocalViewNode(rclcpp::NodeOptions options)
    : Node("LocalView", options)
{
  declare_parameters();
  update_parameters();


  /**
   * @ToDo Wanderer
   * @see https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
   * subscribes the fnc callbackLaser to a topic called "scan"
   * since the simulation is not providing a scan topic /base_scan has to be remapped
   **/
#if MR_VIZ__USE_MY_CODE_UP_TO >= 2
#else
    sub_laser_=this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&LocalViewNode::callback_laser, this, _1));
    sub_cmd_vel_=create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&LocalViewNode::callback_cmd_vel, this, _1));

#endif

  /**
   * @ToDo Wanderer
   * Space for adding stuff to plot the last received command
   **/
#if MR_VIZ__USE_MY_CODE_UP_TO >= 5
#else
    /**
     * @node your code
     **/
#endif

  using namespace std::chrono_literals;
  timer_ = create_wall_timer(
      100ms, std::bind(&LocalViewNode::on_timer, this));
      
  this->on_timer();
  cv::setMouseCallback ( this->get_name(), callback_mouse, this );
}


void LocalViewNode::callback_mouse ( int event, int x, int y, int, void* param ) {
    LocalViewNode* node = reinterpret_cast<LocalViewNode*>(param);
    node->callback_mouse(event, x, y);
}

void LocalViewNode::callback_mouse ( int event, int x, int y) {
    cv::Vec < double, 3 > pw = m2w ( cv::Point ( x,y ) );
    if ( event == cv::EVENT_LBUTTONDOWN ) {
      p_lclick_ = std::make_shared<cv::Vec < double, 3 >>(pw);
      std::cout << pw[0] << ", " << pw[1] << ", " << pw[2] << std::endl;
    }
}

void LocalViewNode::update_transforamtion(){

    /**
     * @ToDo Wanderer
     * you have to fill some local variables which you can use to create the transformation matrix
     * use max_x_, min_x_, max_y_, min_y_, width_pixel_, height_pixel_, rotation_ for the computation
     * This function transforms from the metric space of the sensor coordinate frame to the image coordinate frame
     * Checkout the slides about geometry
     **/

#if MR_VIZ__USE_MY_CODE_UP_TO >= 1
#else
    double dx_ = map_max_x_-map_min_x_;  // visual width
    double dy_ = map_max_y_-map_min_y_;  // visual height
    double sx_ = map_width_pix_/dx_;  // scaling x
    double sy_ = map_height_pix_/dy_;  // scaling y
    double ox_ = -(map_max_x_ + map_min_x_) / 2.0;  // offset image space x
    double oy_ = -(map_max_y_ + map_min_y_) / 2.0;  // offset image space y
    double mx_ = map_width_pix_ / 2.0;  // visual image space x
    double my_ = map_height_pix_ / 2.0;  // visual image space y
    cv::Matx<double, 3, 3 > Tw ( 1, 0, ox_, 0, 1, oy_, 0, 0, 1 ); // translation visual space
    cv::Matx<double, 3, 3 > Sc ( sy_, 0, 0, 0, sx_, 0, 0, 0, 1 ); // scaling
    cv::Matx<double, 3, 3 > Sp ( -1, 0, 0, 0, 1, 0, 0, 0, 1 ); // mirroring
    cv::Matx<double, 3, 3 > R ( cos(map_rotation_+M_PI/2), -sin(map_rotation_+M_PI/2), 0, sin(map_rotation_+M_PI/2), cos(map_rotation_+M_PI/2), 0, 0, 0, 1 );  // rotation
    cv::Matx<double, 3, 3 > Tm ( 1, 0, mx_, 0, 1, my_, 0, 0, 1 ); // translation image space
    Mw2m_ =Tm * R * Sp * Sc * Tw;
    //In general the H⁻1=[R^T -R^t*d; 0 1] but because scaling and mirroring can't use that so we need inverse for each matrix
    cv::Matx<double, 3, 3> Tw_inv(1, 0, -ox_, 0, 1, -oy_, 0, 0, 1); 
    cv::Matx<double, 3, 3> Sc_inv(1.0/sx_, 0, 0, 0, 1.0/sy_, 0, 0, 0, 1); 
    cv::Matx<double, 3, 3> Sp_inv(-1, 0, 0, 0, 1, 0, 0, 0, 1); 
    
    //H⁻1=[R^T -R^t*d; 0 1] 
    cv::Matx<double, 3, 3> R_inv(cos(map_rotation_+M_PI/2), sin(map_rotation_+M_PI/2), 0, -sin(map_rotation_+M_PI/2), cos(map_rotation_+M_PI/2), 0, 0, 0, 1); 
    cv::Matx<double, 3, 3> Tm_inv(1, 0, -mx_, 0, 1, -my_, 0, 0, 1);

    Mm2w_ = Tw_inv * Sc_inv * Sp_inv * R_inv * Tm_inv;
#endif
}


void LocalViewNode::callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  scan_ = msg;
    /**
    * @ToDo Wanderer
    * @see http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
    * creates a callback which fills the laser_measurments_ with laser readings 
    * the readings must be in homogeneous form (x, y, 1) and in robots cartesian coordinate system.
    **/
#if MR_VIZ__USE_MY_CODE_UP_TO >= 3
#else
  laser_measurments_.resize(scan_->ranges.size());

  for (size_t i = 0; i < scan_->ranges.size(); i++) {   
    double angle = scan_->angle_min + (i * scan_->angle_increment);
    
    double d = scan_->ranges[i];
    double x = (d * cos(angle)) + 0.15; //add .15 because the laser has a 15 cm offset
    double y = d * sin(angle);

    laser_measurments_[i] = cv::Vec<double, 3>(x, y, 1.0);
   }

#endif
}

cv::Point LocalViewNode::w2m(const cv::Vec < double, 3 >& p)
{
  cv::Vec < double, 3 > pi = Mw2m_ * p;
  return cv::Point(pi[0],pi[1]);
}
cv::Vec < double, 3 > LocalViewNode::m2w(const cv::Point& p){
  
  return Mm2w_ * cv::Vec < double, 3 >(p.x, p.y, 1);
}

void LocalViewNode::on_timer()
{
  update_transforamtion();
  draw_grid();

  if(!laser_measurments_.empty()){
    std::scoped_lock lock(mutex_);  /// Whats the problem with this lock 
    //mutex looks the for the whole duration that drawing takes 
    //because we use a for which is a very slow operation this can take up a while
    //it would be faster to copy the scan data in the begin and lift the mutex and then 
    //do the whole computation on the copy
   
#if MR_VIZ__USE_MY_CODE_UP_TO >= 4
#else
    for(size_t i =0; i<laser_measurments_.size(); i++){
      cv::circle(view_, w2m(laser_measurments_[i]), 2, cv::Scalar(255, 16, 240) , 1, cv::LINE_AA);
    }
#endif
  } else {
    RCLCPP_INFO(this->get_logger(), "No scan to draw");
  }
  cv::namedWindow(this->get_name(), cv::WINDOW_NORMAL);
  cv::imshow ( this->get_name(), view_ );
  cv::waitKey ( 1 );
}

template<>
void LocalViewNode::declare_default_parameter<int>(
    const std::string &name,
    int value_default,
    int min, int max,int step,
    const std::string &desription)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    rcl_interfaces::msg::IntegerRange range;
    //rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(min).set__to_value(max).set__step(step);
    descriptor.integer_range = {range};
    descriptor.description = desription;
    this->declare_parameter<int>(name, value_default, descriptor);
}

template<>
void LocalViewNode::declare_default_parameter<double>(
    const std::string &name,
    double value_default,
    double min, double max, double step,
    const std::string &desription)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(min).set__to_value(max).set__step(step);
    descriptor.floating_point_range = {range};
    descriptor.description = desription;
    this->declare_parameter<double>(name, value_default, descriptor);
}

void LocalViewNode::declare_parameters()
{
  declare_default_parameter<int>("map_width_pix", 400, 10, 1000, 1, "Map width in pixels");
  declare_default_parameter<int>("map_height_pix", 400, 10, 1000, 1, "Map height in pixels");
  declare_default_parameter<double>("map_max_x",  5., -10., 10., 0.1, "map size in meter");
  declare_default_parameter<double>("map_max_y",  5., -10., 10., 0.1, "map size in meter");
  declare_default_parameter<double>("map_min_x", -5., -10., 10., 0.1, "map size in meter");
  declare_default_parameter<double>("map_min_y", -5., -10., 10., 0.1, "map size in meter");
  declare_default_parameter<double>("map_grid_x", 1., 0., 10., 0.1, "grid size in meter");
  declare_default_parameter<double>("map_grid_y", 1., 0., 10., 0.1, "grid size in meter");
  declare_default_parameter<double>("map_rotation", 0., -3.14, 3.14, 0.01, "map rotation in rad");
  
}

void LocalViewNode::update_parameters()
{
  callback_update_parameters();
  using namespace std::chrono_literals;
  timer_update_parameter_ =
    this->create_wall_timer(
      1000ms, 
      std::bind(&LocalViewNode::callback_update_parameters, this));
}
void LocalViewNode::callback_update_parameters()
{
  this->get_parameter<int>("map_width_pix",  this->map_width_pix_);  
  this->get_parameter<int>("map_height_pix", this->map_height_pix_);  
  this->get_parameter<double>("map_max_x", this->map_max_x_);  
  this->get_parameter<double>("map_max_y", this->map_max_y_);  
  this->get_parameter<double>("map_min_x", this->map_min_x_);  
  this->get_parameter<double>("map_min_y", this->map_min_y_);  
  this->get_parameter<double>("map_grid_x", this->map_grid_x_);  
  this->get_parameter<double>("map_grid_y", this->map_grid_y_); 
  this->get_parameter<double>("map_rotation", this->map_rotation_);  
  // RCLCPP_INFO(this->get_logger(), "Publisher: '%d,%d'", this->map_width_pix_, this->map_height_pix_);
}

void LocalViewNode::draw_grid()
{
  view_.create(map_height_pix_, map_width_pix_, CV_8UC3);
  view_.setTo(0xFF);
  char txt[0xFF];

  if ((map_grid_y_ > 0) && (map_grid_x_ > 0)) {
    cv::Vec < double, 3 > p0, p1;
    p0[2] = p1[2] = 1;
    double min_y = round(map_min_y_ / map_grid_y_) * map_grid_y_;
    double max_y = round(map_max_y_ / map_grid_y_) * map_grid_y_;
    for (p0[1] = min_y; p0[1] <= max_y; p0[1] += map_grid_y_) {
      p0[0] = round(map_max_x_ / map_grid_x_) * map_grid_x_;
      p1[1] = p0[1];
      p1[0] = round(map_min_x_ / map_grid_x_) * map_grid_x_;
      if (fabs(p0[1]) > FLT_MIN) {
        cv::line(view_, w2m(p0), w2m(p1), cv::Scalar(244,244,244), 1, cv::LINE_AA);
      } else {
        cv::line(view_, w2m(p0), w2m(p1), cv::Scalar(128,128,128), 1, cv::LINE_AA);
      }
    }

    p0 = cv::Vec < double, 3 >(0, max_y - map_grid_y_ / 2.0, 1.);
    sprintf(txt, "y = %3.2f", max_y);
    cv::putText(view_, txt, w2m(p0), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(255,255,255), 3, cv::LINE_AA);
    cv::putText(view_, txt, w2m(p0), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(128,128,128), 1, cv::LINE_AA);
    p1 = cv::Vec < double, 3 >(0, min_y + map_grid_y_ / 2.0, 1.);
    sprintf(txt, "y = %3.2f", min_y);
    cv::putText(view_, txt, w2m(p1), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(255,255,255), 5, cv::LINE_AA);
    cv::putText(view_, txt, w2m(p1), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(128,128,128), 1, cv::LINE_AA);


    double min_x = round(map_min_x_ / map_grid_x_) * map_grid_x_;
    double max_x = round(map_max_x_ / map_grid_x_) * map_grid_x_;
    for (p0[0] = min_x; p0[0] <= max_x; p0[0] += map_grid_x_) {
      p0[1] = round(map_max_y_ / map_grid_y_) * map_grid_y_;
      p1[0] = p0[0];
      p1[1] = round(map_min_y_ / map_grid_y_) * map_grid_y_;
      if (fabs(p0[0]) > FLT_MIN) {
        cv::line(view_, w2m(p0), w2m(p1), cv::Scalar(244,244,244), 1, cv::LINE_AA);
      } else {
        cv::line(view_, w2m(p0), w2m(p1), cv::Scalar(128,128,128), 1, cv::LINE_AA);
      }
    }
    p0 = cv::Vec < double, 3 >(max_x - map_grid_x_ / 2.0, 0., 1.);
    sprintf(txt, "x = %3.2f", max_x);
    cv::putText(view_, txt, w2m(p0), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(255,255,255), 5, cv::LINE_AA);
    cv::putText(view_, txt, w2m(p0), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(128,128,128), 1, cv::LINE_AA);
    p1 = cv::Vec < double, 3 >(min_x + map_grid_x_ / 2.0, 0., 1.);
    sprintf(txt, "x = %3.2f", min_x);
    cv::putText(view_, txt, w2m(p1), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(255,255,255), 3, cv::LINE_AA);
    cv::putText(view_, txt, w2m(p1), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(128,128,128), 1, cv::LINE_AA);
   
    if(p_lclick_) {
      sprintf(txt, "%+3.2f, %+3.2f", (*p_lclick_)[0], (*p_lclick_)[1]);
      cv::putText(view_, txt, cv::Point(10,10), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(255,255,255), 3, cv::LINE_AA);
      cv::putText(view_, txt, cv::Point(10,10), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(128,128,128), 1, cv::LINE_AA);
    }
    cv::putText(view_, "Jana Grabher", cv::Point(10,map_height_pix_-10), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(128,128,128), 1, cv::LINE_AA);


  /**
   * @ToDo Wanderer
   * Space for adding stuff to plot the last received command
   **/
#if MR_VIZ__USE_MY_CODE_UP_TO >= 5
#else
   //open CV can do format directly 
  char temp_text[100]; 
  sprintf(temp_text, "v = %.2f m/s", v);
  cv::putText(view_,temp_text , cv::Point(map_width_pix_ - 50, 10), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(255,0,0), 1, cv::LINE_AA);
  sprintf(temp_text, "w = %.2f rad/s", w);
  cv::putText(view_, temp_text, cv::Point(map_width_pix_-50,30), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(0,0,255), 1, cv::LINE_AA);
#endif

   }
}

  /**
   * @ToDo Wanderer
   * Space for adding functions to assist with other tasks
   **/
#if MR_VIZ__USE_MY_CODE_UP_TO >= 1
#else
void LocalViewNode::callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  twist_=msg; //Save last twist message
  v=twist_->linear.x;
  w=twist_->angular.z;
}
#endif

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(LocalViewNode)
