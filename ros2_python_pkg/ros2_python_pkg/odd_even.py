#!/usr/bin/env python3
import rclpy   
from rclpy.node import Node    
from std_msgs.msg import String, Int32

class SubPubNode(Node):
    def __init__(self):
        super().__init__("odd_even_node")     
        self.pub = self.create_publisher(String, "/odd_even", qos_profile=10)
        self.timer = self.create_timer(0.5, self.publisher_callback)
        self.sub = self.create_subscription(Int32, "/random_num", self.subscriber_callback, qos_profile=10)
        self.number = ""
        

    def subscriber_callback(self, num):
        # self.get_logger().info('"Recieved Number: "%s"' % num.data)
        self.number = num.data


    def publisher_callback(self):   
        msg = String()  
        temp = int(self.number)
        if (temp %2 == 0):
            msg.data = "EVEN"
        else:
            msg.data = "ODD"
        self.pub.publish(msg)   
        self.get_logger().info('Number is: "%s"' % msg.data)	  


def main(args=None):
	rclpy.init(args=args)     
	my_sub = SubPubNode()     
	print("Waiting for data to be published...")

	try:
		rclpy.spin(my_sub)     
	except KeyboardInterrupt:      
		print("Terminating Node...")
		my_sub.destroy_node()      
		rclpy.shutdown()     


if __name__ == '__main__':
	main()