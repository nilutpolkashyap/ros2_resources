#include <iostream>
#include "rclcpp/rclcpp.hpp"   //ROS 2 client library for C++
#include "std_msgs/msg/string.hpp"    //import built-in string message type from ROS data types


class HelloWorldSubNode : public rclcpp::Node   //creating a new class  inheriting from rclcpp Node class
{ 
  public: 
    HelloWorldSubNode() : Node("hello_world_sub_node")   // node name
		{
			subscription_ = this->create_subscription<std_msgs::msg::String>(
				"hello_world", 10, std::bind(&HelloWorldSubNode::sub_callback, this, std::placeholders::_1)
			);
		}

	private:
		void sub_callback(const std_msgs::msg::String & msg) const
		{
			std::cout << msg.data << std::endl;
		}
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);   // Initialize the rclcpp library
  rclcpp::spin(std::make_shared<HelloWorldSubNode>());   // allowing the ros node to keep running
  rclcpp::shutdown();    // shutdown rclpy instances
  
  return 0;
}
