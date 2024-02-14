#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Create a ROS node
  auto node = std::make_shared<rclcpp::Node>("my_node");

  // Add some code here to implement your ROS functionality

  // Wait for ROS to finish spinning
  rclcpp::spin(node);

  // Shutdown ROS
  rclcpp::shutdown();

  return 0;
}
