#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64

MY_NUMBER_DEFAULT = 250.00

class ParamsExamplePublisher(Node):
    def __init__(self):
        super().__init__("hello_world_publisher222")  
        self.pub = self.create_publisher(Float64, "/hello_world", 10)
        self.declare_parameter("my_number", MY_NUMBER_DEFAULT)
        self.timer = self.create_timer(0.5, self.publisher_callback)
        self.counter  = 0

        print(self.get_parameter("my_number").get_parameter_value())

    def publisher_callback(self):
        new_number = self.get_parameter("my_number").get_parameter_value().double_value
        radius = 2 * 3.14 * new_number
        msg = Float64()
        msg.data = radius
        self.pub.publish(msg)
        # print('Publishing : ', msg.data)
        self.get_logger().info('Publishing : "%f"' % msg.data)
        self.counter +=1 

def main(args=None):
    rclpy.init()
    my_pub = ParamsExamplePublisher()

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        print("Terminating node")
        my_pub.destroy_node()

if __name__ == '__main__':
    main()