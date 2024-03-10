#include "rclcpp/rclcpp.hpp"        // ROS 2 C++ library
#include "sensor_msgs/msg/image.hpp" // ROS 2 message type for images
#include "opencv2/opencv.hpp"       // OpenCV library for image processing
#include "cv_bridge/cv_bridge.h"    // Package for converting between OpenCV and ROS image formats

class WebcamSubscriber : public rclcpp::Node
{
public:
    // Constructor for the WebcamSubscriber class
    WebcamSubscriber() : Node("webcam_subscriber")
    {
        // Create a subscriber for 'sensor_msgs/Image' messages on the topic '/webcam/image'
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "webcam/image", 10, std::bind(&WebcamSubscriber::image_callback, this, std::placeholders::_1)
        );
    }

private:
    // Callback function for handling incoming image messages
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Convert the ROS 'sensor_msgs/Image' message to an OpenCV image
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            RCLCPP_INFO(get_logger(), "Receiving webcam video frame");
            // Display the image
            cv::imshow("Webcam Feed", frame);
            cv::waitKey(1);
        }
        catch (const cv_bridge::Exception& e)
        {
            // Handle cv_bridge exception
            // RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());   //Assertion failed error here, needs to be fixed
            RCLCPP_ERROR(this->get_logger(), "An error occurred while processing the image");
        }
    }

    // Subscriber for 'sensor_msgs/Image' messages
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

// Main function
int main(int argc, char *argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create an instance of the 'WebcamSubscriber' node
    rclcpp::spin(std::make_shared<WebcamSubscriber>());

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();

    return 0;
}
