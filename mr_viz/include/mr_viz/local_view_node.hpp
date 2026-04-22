#ifndef MR_VIZ_PKG__LOCAL_VIEW_NODE_HPP_
#define MR_VIZ_PKG__LOCAL_VIEW_NODE_HPP_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

/**
 * Class for to visualize sensor readings with an OpenCV window
 */
class LocalViewNode : public rclcpp::Node
{
public:
    /// Constructor
  __attribute__ ((visibility("default"))) LocalViewNode(rclcpp::NodeOptions options);

  /// Callback function for opencv window (static version)
  static void callback_mouse ( int event, int x, int y, int flags, void* param );
private:

  int map_width_pix_;            /// map width in pixels
  int map_height_pix_;           /// map height in pixels
  double map_max_x_;             /// max x value to show on the map [m]
  double map_max_y_;             /// max y value to show on the map [m]
  double map_min_x_;             /// min x value to show on the map [m]
  double map_min_y_;             /// min y value to show on the map [m]
  double map_grid_x_;            /// grid x size to draw [m]
  double map_grid_y_;            /// grid y size to draw [m]
  double map_rotation_;          /// rotation of the map [rad] -pi ... +pi

  cv::Matx33d Mw2m_;             /// transformation world to map
  cv::Matx33d Mm2w_;             /// transformation map to world

  cv::Mat view_;                 /// opencv window to draw the measurment plot
  std::mutex mutex_;             /// mutex to ensure mutually exclusive access on data
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_; /// laser subscriber
  sensor_msgs::msg::LaserScan::SharedPtr scan_;  /// local copy of the last scan
  rclcpp::TimerBase::SharedPtr timer_; /// timer to on_time()
  std::shared_ptr<cv::Vec < double, 3 >> p_lclick_;      /// if allocated a left clicked point
  std::vector<cv::Vec < double, 3 > > laser_measurments_;  /// laser measurments in cartesian robot coordinates

  /// Callback function for opencv window (member version)
  void callback_mouse ( int event, int x, int y);
  
  /// Callback function for incomming range measurments
  void callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /// function called by an internal callback x time per second 
  void on_timer();

  /**
  * Converts a point from world coodinates [m] to map coordinates [pix] for drawing
  * @param p point in meter 
  * @return point in pixel
  */
  cv::Point w2m(const cv::Vec < double, 3 >& p);       
  
  /**
  * Converts a point from map coordinates [pix] to world coodinates [m]
  * @param p point in pixel 
  * @return point in meter
  */ 
  cv::Vec < double, 3 > m2w(const cv::Point& p);

  void draw_grid();                                     /// draws the grind on the view
  void update_transforamtion();                         /// updates the transformation matrix
  void declare_parameters();                            /// declares ros parameters
  void update_parameters();                             /// int ros parameters for the startup
  void callback_update_parameters();                    /// callback to check changes on the parameters
  rclcpp::TimerBase::SharedPtr timer_update_parameter_; /// timer to check regulary for parameter changes

  /**
   * Templated to declare paramters with different types
   * @param name
   * @param value_default
   * @param min
   * @param max
   * @param step
   * @param desription
  */
  template<typename T> void declare_default_parameter(
    const std::string &name,
    T value_default,
    T min, T max, T step,
    const std::string &desription);

  /**
   * @ToDo Wanderer
   * Space for adding stuff to plot the last received command
   **/
#if MR_VIZ__USE_MY_CODE_UP_TO >= 5
#else
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  void callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  geometry_msgs::msg::Twist::SharedPtr twist_; 
  double v = 0.0;
  double w = 0.0;
#endif

};

#endif  // MR_VIZ_PKG__LOCAL_VIEW_NODE_HPP_
