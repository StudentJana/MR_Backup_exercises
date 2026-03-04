#include <memory>
#include "mr_viz/local_view_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalViewNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
