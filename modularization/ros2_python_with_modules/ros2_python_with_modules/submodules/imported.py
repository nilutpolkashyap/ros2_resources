from rclpy.node import Node
from std_msgs.msg import String
import rclpy

class MyPublisher:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(String, 'my_topic', 10)
        self.i = 0

    def start_publishing(self):
        # Create and configure a timer
        timer_period = 0.5  
        self.timer = self.node.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello, ROS 2 world! {self.i}"
        self.publisher.publish(msg)
        self.node.get_logger().info(f"Published: {msg.data}")
        self.i += 1
