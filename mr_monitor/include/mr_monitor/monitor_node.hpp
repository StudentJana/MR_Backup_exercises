#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include "rclcpp/rclcpp.hpp"

class MonitorNode;

  class Robot {
    public: 
    Robot(MonitorNode *monitor, size_t id, int map_size);
    MonitorNode *monitor_;
    size_t id_;
    int map_size_;
    std::string name_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ground_truth_;
    void callback_ground_truth(const nav_msgs::msg::Odometry::SharedPtr);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_name_;
    std::vector<int> visited;
  };


class MonitorNode : public rclcpp::Node
{
public:
  MonitorNode(rclcpp::NodeOptions options);

private:
  void on_timer();

  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  std::vector<std::shared_ptr<Robot>> robots;

  void plot();

  int enviroment_size_;
  size_t nr_of_robots_;
  cv::Mat figure_;
};
