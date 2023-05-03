#!/usr/bin/env python3
import rclpy   
from rclpy.node import Node    
from std_msgs.msg import String, Int32   
import random

class RandomNumberPublisher(Node):    
	def __init__(self):
		super().__init__("random_num_node")     
		self.pub = self.create_publisher(Int32, "/random_num", qos_profile=10)
		self.timer = self.create_timer(0.5, self.random_num_publisher)
		self.counter = 0   

	def random_num_publisher(self):    
		msg = Int32()    
		msg.data = random.randint(0, 10)
		self.pub.publish(msg)     
		self.get_logger().info('Publishing Random Number: "%s"' % msg.data)	  
		self.counter += 1     

def main(args=None):
	rclpy.init(args=args)     
	my_pub = RandomNumberPublisher()     
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