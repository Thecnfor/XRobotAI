// /opt/projects/ros/src/navigat/src/navigator_node.cpp
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("navigator_node");
  RCLCPP_INFO(node->get_logger(), "navigator_node started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}