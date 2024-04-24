import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from my_custom_interfaces.action import Example    # Importing the Example action message

class ExampleActionServer(Node):
    def __init__(self):
        # Initialize the node with the name 'example_action_server'
        super().__init__('example_action_server')
        # Log a message indicating that the action server has started
        self.get_logger().info('Example action server has started')
        # Create an action server for the Example action
        self._action_server = ActionServer(
            self,
            Example,
            'example',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        # Log a message indicating that a goal request has been received
        self.get_logger().info('Goal Request received: {0} . Executing goal...'.format(goal_handle.request.number))

        # Create a feedback message
        feedback_msg = Example.Feedback()
        feedback_msg.status = 0

        # Iterate from 1 to the requested number and publish feedback
        for i in range(1, goal_handle.request.number + 1):
            feedback_msg.status = i
            # Log feedback status
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.status))
            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            # Wait for 1 second
            time.sleep(1)
        # Indicate that the goal has succeeded
        goal_handle.succeed()

        # Create and return the result message
        result = Example.Result()
        result.finalresult = True
        return result

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the ExampleActionServer class
    example_action_server = ExampleActionServer()
    # Spin the node until shutdown is requested
    rclpy.spin(example_action_server)


if __name__ == '__main__':
    main()
