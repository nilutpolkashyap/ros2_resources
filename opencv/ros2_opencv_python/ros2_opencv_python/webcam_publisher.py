import cv2  # OpenCV library for image processing
import rclpy
from rclpy.node import Node  
from sensor_msgs.msg import Image, CameraInfo  # ROS 2 message type for images
from cv_bridge import CvBridge  # Package for converting between OpenCV and ROS image formats

class WebcamPublisher(Node):
    def __init__(self):
        try:
            super().__init__('webcam_publisher')  # Initialize the ROS 2 node with the name 'webcam_publisher'
            self.publisher_ = self.create_publisher(Image, 'webcam/image', 10)  # Create a publisher for the 'sensor_msgs/Image' messages on the topic '/webcam/image'
            self.camera_info_publisher = self.create_publisher(CameraInfo, 'webcam/camera_info', 10)  # Create a publisher for camera info messages on the topic '/webcam/camera_info'
            self.bridge = CvBridge()  # Initialize the OpenCV to ROS bridge
            self.capture = cv2.VideoCapture("/home/user/colcon_ws/src/GICWmACN6Hhns8QCAGcvFBwce5kYbpR1A.mp4")  # Open the default webcam (index 0)

            # Check if the webcam is opened successfully
            if not self.capture.isOpened():
                self.get_logger().error('Error: Could not open webcam.')
                raise Exception('Could not open webcam.')

            # Create a timer that triggers the 'publish_frame' method every 0.1 seconds
            self.timer = self.create_timer(0.1, self.publish_frame)
        except Exception as e:
            self.get_logger().error(f"An error occurred during initialization: {str(e)}")
            rclpy.shutdown()

    def publish_frame(self):
        try:    
            ret, frame = self.capture.read()  # Read a frame from the webcam
            if ret:
                # Convert the OpenCV frame to a ROS 'sensor_msgs/Image' message
                image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                # Create a camera info message
                camera_info_msg = self.create_camera_info(image_msg)
                # Publish the 'sensor_msgs/Image' message on the topic '/webcam/image'
                self.publisher_.publish(image_msg)
                self.get_logger().info('Publishing webcam video frame')
                # Publish the 'sensor_msgs/CameraInfo' message on the topic '/webcam/camera_info'
                self.camera_info_publisher.publish(camera_info_msg)
        except Exception as e:
            self.get_logger().error(f"An error occurred while publishing frame: {str(e)}")

    def create_camera_info(self, image_msg):
        # Create a simple 'sensor_msgs/CameraInfo' message
        camera_info = CameraInfo()
        camera_info.header = image_msg.header
        camera_info.width = image_msg.width
        camera_info.height = image_msg.height
        camera_info.distortion_model = 'plumb_bob'
        # Populate other camera info parameters as needed

        return camera_info

def main(args=None):
    try:
        rclpy.init(args=args)  # Initialize the ROS 2 Python client library
        webcam_publisher = WebcamPublisher()  # Create an instance of the 'WebcamPublisher' node
        rclpy.spin(webcam_publisher)  # Spin the node, blocking until ROS 2 is shutdown
        webcam_publisher.destroy_node()  # Clean up resources when the node is destroyed
        # rclpy.shutdown()  # Shutdown the ROS 2 client library
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:
        rclpy.shutdown()  # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()  
