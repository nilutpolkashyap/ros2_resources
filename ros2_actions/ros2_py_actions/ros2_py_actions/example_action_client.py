import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_custom_interfaces.action import Example    # Importing the Example action message

class ExampleActionClient(Node):
    def __init__(self):
        # Initialize the node with the name 'example_action_client'
        super().__init__('example_action_client')
        # Create an action client for the Example action
        self._action_client = ActionClient(self, Example, 'example')

    def send_goal(self, number):
        # Create a goal message with the provided number
        goal_msg = Example.Goal()
        goal_msg.number = number
        # Wait for the action server to become available
        self._action_client.wait_for_server()
        # Send the goal request to the action server asynchronously
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        # Log the goal request
        self.get_logger().info('Goal Request sent: {0} '.format(number))
        # Add a callback to handle the goal response
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Retrieve the goal handle from the future
        goal_handle = future.result()
        # Check if the goal was accepted by the server
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        # Log that the goal was accepted
        self.get_logger().info('Goal accepted :)')
        # Get the result asynchronously and add a callback to handle it
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Retrieve the result from the future
        result = future.result().result
        # Log the final result received from the action server
        self.get_logger().info('Result: {0}'.format(result.finalresult))
        # Shutdown the ROS 2 client library
        rclpy.shutdown()


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    # Create an instance of the ExampleActionClient class
    action_client = ExampleActionClient()

    # Send a goal request with a number argument
    action_client.send_goal(10)
    # Spin the node until shutdown is requested
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
