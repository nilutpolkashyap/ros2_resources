#!/usr/bin/env python3
import rclpy   
from rclpy.node import Node    
from std_msgs.msg import String   
from my_custom_interfaces.msg import HardwareStatus

class HardwareStatusPublisher(Node):    
	def __init__(self):
		super().__init__("hello_world_pub_node")     
		self.pub = self.create_publisher(HardwareStatus, "hello_world", qos_profile=10)

		self.timer = self.create_timer(0.5, self.publish_hello_world)
		self.counter = 0   

	def publish_hello_world(self):    
		msg = HardwareStatus()     
		msg.debug_message = "This is a debug message"
		msg.temperature = 15
		self.pub.publish(msg)     
		self.get_logger().info('Publishing: ')	
		self.counter += 1    

def main(args=None):
	rclpy.init(args=args)     
	my_pub = HardwareStatusPublisher()     
	print("Publisher Node Running...")

	try:
		rclpy.spin(my_pub)   
	except KeyboardInterrupt:      
		print("Terminating Node...")
		my_pub.destroy_timer(timer=my_pub.timer)  
		my_pub.destroy_node()              
		rclpy.shutdown()     


if __name__ == '__main__':
	main()