from my_custom_interfaces.srv import RectangleArea  # Importing the RectangleArea service message
import rclpy  
from rclpy.node import Node  

class RectangleAreaService(Node):
    def __init__(self):
        # Initializing the node with the name 'rectangle_area_service'
        super().__init__('rectangle_area_service')  
        # Creating the service with the name 'rectangle_area' and specifying the callback function
        self.srv = self.create_service(RectangleArea, 'rectangle_area', self.rectangle_area_callback)  
        self.get_logger().info('Rectangle area calculate server has started')  # Logging a message indicating that the server has started

    def rectangle_area_callback(self, request, response):
        # Calculating the area using the length and width from the request
        response.area = request.length * request.width  
        # Logging the incoming request with its length and width
        self.get_logger().info('Incoming request\nlength: %.2f width: %.2f' % (request.length, request.width))  
        # Logging the calculated area before sending the response
        self.get_logger().info('Sent the Result calculated area: %.2f' % response.area)  
        # Returning the response
        return response  

def main():
    # Initializing the ROS 2 client library
    rclpy.init()  
    # Creating an instance of the RectangleAreaService class
    rectangle_area_service = RectangleAreaService()  
    # Spinning the node to handle service requests
    rclpy.spin(rectangle_area_service)  
    # Shutting down the ROS 2 client library
    rclpy.shutdown()  

if __name__ == '__main__':
    main()  
