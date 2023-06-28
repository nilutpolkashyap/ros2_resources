#include <chrono>
#include <functional> 
#include "rclcpp/rclcpp.hpp"   //ROS 2 client library for C++
#include "std_msgs/msg/string.hpp"   //import built-in string message type from ROS data types

using namespace std::chrono_literals;

class HelloWorldPubNode : public rclcpp::Node  // creating a new class  inheriting from rclcpp Node class
{
  public: 
    HelloWorldPubNode() : Node("hello_world_pub_node")   // ==> node name
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("hello_world", 10);
      timer_ = this->create_wall_timer(1s, std::bind(&HelloWorldPubNode::publish_hello_world, this));
    }

  private:
    void publish_hello_world()    //callback function ==> call this function from within your code – here as a callback in a timer
    {
      auto message = std_msgs::msg::String();   //create String message object
      message.data = "Hello World  " + std::to_string(counter_);
      publisher_->publish(message);   // publish() function from the publisher object to actually publish on the topic
      counter_++;    //incrementing counter variable
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t counter_ = 0;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);    // Initialize the rclcpp library
  rclcpp::spin(std::make_shared<HelloWorldPubNode>());    // allowing the ros node to keep running
  rclcpp::shutdown();    // shutdown rclpy instances
  
  return 0;
}
