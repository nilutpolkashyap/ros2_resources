#include "rclcpp/rclcpp.hpp"
#include "ros2_cpp_with_headers/MyLibrary.hpp"
#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_cpp_with_headers::MyLibrary>();
  RCLCPP_INFO(node->get_logger(), "Using ROS 2 Library Header in C++");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}