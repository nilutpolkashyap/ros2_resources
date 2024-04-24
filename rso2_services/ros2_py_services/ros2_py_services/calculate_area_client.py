import sys
from my_custom_interfaces.srv import RectangleArea  # Importing the RectangleArea service message
import rclpy
from rclpy.node import Node

class RectangleAreaClientAsync(Node):
    def __init__(self):
        # Initialize the node with the name 'rectangle_area_client_async'
        super().__init__('rectangle_area_client_async')
        # Create a client for the RectangleArea service
        self.cli = self.create_client(RectangleArea, 'rectangle_area')
        # Wait for the service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        # Create a request object for the service
        self.req = RectangleArea.Request()

    def send_request(self, length, width):
        # Set the length and width parameters in the request object
        self.req.length = length
        self.req.width = width
        # Call the service asynchronously and wait for the response
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    # Check if command line arguments are provided
    if len(sys.argv) != 3:
        print("Usage: python3 rectangle_area_client.py <length> <width>")
        sys.exit(1)

    # Initialize the ROS 2 Python client library
    rclpy.init()
    # Create an instance of the RectangleAreaClientAsync class
    rectangle_area_client = RectangleAreaClientAsync()
    # Send a request to the service with the length and width parameters
    response = rectangle_area_client.send_request(float(sys.argv[1]), float(sys.argv[2]))
    # Log the result of the service call
    rectangle_area_client.get_logger().info(
        'Result of rectangle_area service: for length=%.2f and width=%.2f, area=%.2f' %
        (float(sys.argv[1]), float(sys.argv[2]), response.area))

    # Destroy the node
    rectangle_area_client.destroy_node()
    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
