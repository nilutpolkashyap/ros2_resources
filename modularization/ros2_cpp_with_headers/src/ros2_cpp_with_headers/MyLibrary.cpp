#include "ros2_cpp_with_headers/MyLibrary.hpp"
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <utility>

namespace ros2_cpp_with_headers {

using std::placeholders::_1;

MyLibrary::MyLibrary() : Node("my_publisher") {
  pub_ = this->create_publisher<std_msgs::msg::String>("/hello_world", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                   std::bind(&MyLibrary::TimerCallback, this));
}

void MyLibrary::TimerCallback() {
  std_msgs::msg::String message;
  message.data = "Hello, ROS 2 world! " + std::to_string(counter_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  pub_->publish(message);
}

} // namespace ros2_cpp_with_headers