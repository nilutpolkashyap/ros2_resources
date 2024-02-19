from .submodules.imported import MyPublisher
from rclpy.node import Node
import rclpy

def main():
    rclpy.init()
    node = Node('my_publisher')

    my_publisher = MyPublisher(node)
    my_publisher.start_publishing()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
