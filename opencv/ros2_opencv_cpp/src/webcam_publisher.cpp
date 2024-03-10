#include "rclcpp/rclcpp.hpp"        // ROS 2 C++ library
#include "sensor_msgs/msg/image.hpp" // ROS 2 message type for images
#include "sensor_msgs/msg/camera_info.hpp" // ROS 2 message type for camera info
#include "opencv2/opencv.hpp"       // OpenCV library for image processing
#include "cv_bridge/cv_bridge.h"    // Package for converting between OpenCV and ROS image formats

class WebcamPublisher : public rclcpp::Node
{
public:
    // Constructor for the WebcamPublisher class
    WebcamPublisher() : Node("webcam_publisher")
    {
        // Create a publisher for 'sensor_msgs/Image' messages on the topic '/webcam/image'
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("webcam/image", 10);
        // Create a publisher for 'sensor_msgs/CameraInfo' messages on the topic '/webcam/camera_info'
        camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("webcam/camera_info", 10);

        // Create a timer that triggers the 'publish_frame' method every 0.1 seconds
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&WebcamPublisher::publish_frame, this));

        // Open the default webcam (index 0)
        capture_ = cv::VideoCapture(0);

        // Check if the webcam is opened successfully
        if (!capture_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Could not open webcam.");
            rclcpp::shutdown();
        }

        // Set camera parameters (modify as needed)
        camera_info_msg_.header.frame_id = "webcam_frame";
        camera_info_msg_.width = static_cast<uint32_t>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));
        camera_info_msg_.height = static_cast<uint32_t>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
        camera_info_msg_.k[0] = camera_info_msg_.k[4] = 1.0; // focal length (assume normalized image)
        camera_info_msg_.k[2] = camera_info_msg_.k[5] = camera_info_msg_.p[2] = camera_info_msg_.p[6] = 0.0;
        camera_info_msg_.distortion_model = "plumb_bob";
    }

private:
    // Method to publish a frame from the webcam
    void publish_frame()
    {
        cv::Mat frame;
        // Read a frame from the webcam
        capture_ >> frame;

        // Check if the frame is not empty
        if (!frame.empty())
        {
            // Convert the OpenCV frame to a ROS 'sensor_msgs/Image' message
            auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            // Publish the 'sensor_msgs/Image' message on the topic '/webcam/image'
            image_publisher_->publish(std::make_unique<sensor_msgs::msg::Image>(*image_msg));

            RCLCPP_INFO(get_logger(), "Publishing webcam video frame");

            // Update timestamp for CameraInfo message
            camera_info_msg_.header.stamp = this->now();
            // Publish the 'sensor_msgs/CameraInfo' message on the topic '/webcam/camera_info'
            camera_info_publisher_->publish(std::make_unique<sensor_msgs::msg::CameraInfo>(camera_info_msg_));
        }
    }

    // Publishers for image and camera info messages
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    // Timer for triggering frame publishing
    rclcpp::TimerBase::SharedPtr timer_;
    // Object for capturing frames from the webcam
    cv::VideoCapture capture_;
    // Camera info message
    sensor_msgs::msg::CameraInfo camera_info_msg_;
};

// Main function
int main(int argc, char *argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);
    // Create an instance of the 'WebcamPublisher' node
    rclcpp::spin(std::make_shared<WebcamPublisher>());
    // Shutdown the ROS 2 client library
    rclcpp::shutdown();
    return 0;
}
