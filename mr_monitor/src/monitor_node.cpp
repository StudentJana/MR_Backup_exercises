

#include "mr_monitor/monitor_node.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std::chrono_literals;

using std::placeholders::_1;

cv::Scalar colors[] = {cv::Scalar(0, 0, 255),
            cv::Scalar(47, 255, 173),
            cv::Scalar(47, 107, 85),
            cv::Scalar(255, 0, 255),
            cv::Scalar(255, 255, 0),
            cv::Scalar(0, 255, 255),
            cv::Scalar(143, 188, 143)};

Robot::Robot(MonitorNode *monitor, size_t id, int map_size)
    : monitor_(monitor), id_(id), map_size_(map_size)
{
  std::string topic_ground_truth = std::string("/robot_") + std::to_string(id_) + std::string("/ground_truth");
  name_ = std::string("robot_") + std::to_string(id_);
  visited.resize(map_size_ * map_size_, 0);
  sub_ground_truth_ = monitor_->create_subscription<nav_msgs::msg::Odometry>(
      topic_ground_truth,
      10, std::bind(&Robot::Robot::callback_ground_truth, this, _1));

  std::string topic_name = std::string("/robot_") + std::to_string(id_) + std::string("/name");
  auto callback_name =
    [this](std_msgs::msg::String::UniquePtr msg) -> void {
      name_ = msg->data;
    };
  sub_name_ = monitor_->create_subscription<std_msgs::msg::String>(topic_name, 10, callback_name);
}

void Robot::callback_ground_truth(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  int c = round(odom->pose.pose.position.x) + map_size_ / 2;
  int r = round(odom->pose.pose.position.y) + map_size_ / 2;
  int idx = r * map_size_ + c;
  if ((idx <= map_size_ * map_size_) && (idx >= 0))
  {
    RCLCPP_INFO(monitor_->get_logger(), "robot %zu on %d, %d", id_, c, r);
    visited[idx] = 1;
  }
  else
  {
    RCLCPP_INFO(monitor_->get_logger(), "robot %zu is outside", id_);
  }
}

MonitorNode::MonitorNode(rclcpp::NodeOptions options)
    : Node("Monitor", options), enviroment_size_(20), nr_of_robots_(7)
{

  for (size_t i = 0; i < nr_of_robots_; i++)
  {
    auto r = std::make_shared<Robot>(this, i, enviroment_size_);
    robots.push_back(r);
  }

    cv::namedWindow ( "Monitor", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_NORMAL );
    figure_.create ( 200, 400, CV_8UC3 );

  // Call on_timer function every second
  timer_ = this->create_wall_timer(
      1s, [this]()
      { return this->on_timer(); });
}

void MonitorNode::on_timer()
{
  RCLCPP_INFO(this->get_logger(), "on_timer");
	plot();
}

void MonitorNode::plot() {
  
  figure_.setTo ( 0xFF );
  int space = 20;
  int bar_height = 10;
  int name_max = 100;
  char text[0xff];
  for(auto &robot: robots){
    int sum_of_elems = std::accumulate(robot->visited.begin(), robot->visited.end(), 0);
    cv::Rect bar(70,space*(robot->id_+1), sum_of_elems*2, bar_height+1);
    sprintf(text, "%zu %i", robot->id_, sum_of_elems);
    int line = space*(robot->id_ +1)+bar_height;
    cv::putText(figure_, text, cv::Point(5,line), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0), 1, cv::LINE_AA);
    cv::rectangle(figure_, bar, colors[robot->id_], cv::FILLED);

    // robot name
    bar.x = figure_.cols - name_max;
    bar.width = name_max;
    cv::rectangle(figure_, bar, colors[robot->id_], cv::FILLED);
    cv::putText(figure_, robot->name_.c_str(), cv::Point(figure_.cols - name_max, line), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0), 1, cv::LINE_AA); 
  }
  cv::imshow ( "Monitor", figure_ );
  cv::waitKey ( 1 );
}