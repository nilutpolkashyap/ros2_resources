import rclpy  
from rclpy.node import Node  
from sensor_msgs.msg import Image  # ROS 2 message type for images
from cv_bridge import CvBridge  # Package for converting between OpenCV and ROS image formats
import cv2  # OpenCV library for image processing

class WebcamSubscriber(Node):
    def __init__(self):
        super().__init__('webcam_subscriber')  # Initialize the ROS 2 node with the name 'webcam_subscriber'
        self.subscription = self.create_subscription(Image, 'webcam/image', self.callback, 10)  # Create a subscriber for the 'sensor_msgs/Image' messages on the topic '/webcam/image'
        self.subscription  # Prevent unused variable warning
        self.bridge = CvBridge()  # Initialize the OpenCV to ROS bridge

    def callback(self, msg):
        try:
            # Convert the ROS 'sensor_msgs/Image' message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.get_logger().info('Receiving webcam video frame')
            # Display the image
            cv2.imshow('Webcam Feed', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"An error occurred while processing the image: {str(e)}")

def main(args=None):
    try:
        rclpy.init(args=args)  # Initialize the ROS 2 Python client library
        webcam_subscriber = WebcamSubscriber()  # Create an instance of the 'WebcamSubscriber' node
        rclpy.spin(webcam_subscriber)  # Spin the node, blocking until ROS 2 is shutdown
        webcam_subscriber.destroy_node()  # Clean up resources when the node is destroyed
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:
        rclpy.shutdown()  # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()  # Run the 'main' function if the script is executed directly
