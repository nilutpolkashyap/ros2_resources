#ifndef ROS2_CPP_WITH_HEADERS_MY_LIBRARY_HPP
#define ROS2_CPP_WITH_HEADERS_MY_LIBRARY_HPP

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace ros2_cpp_with_headers {

using namespace std::chrono_literals; // NOLINT

class MyLibrary : public rclcpp::Node {
public:
  MyLibrary();

private:
  size_t counter_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  void TimerCallback();
};

} // namespace ros2_cpp_with_headers

#endif // ROS2_CPP_WITH_HEADERS_MY_LIBRARY_HPP