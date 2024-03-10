import rclpy
# from rclpy.node import Node
from rclpy.lifecycle import State, TransitionCallbackReturn, Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class MyLifecycleNode(Node):
    def __init__(self, node_name, **kwargs):
        self._count: int = 0
        
        # Initialize the lifecycle node
        super().__init__(node_name, **kwargs)

    # Callback function for walltimer
	# This function gets invoked by the timer and executes the publishing
    def callback(self):
        """Publish a new message when enabled."""
        msg = String()
        msg.data = 'Hello World -> ' + str(self._count)
        self._count += 1

        ## Print the current state of lifecycle publisher
        if self._lifecycle_pub.is_activated:
            self.get_logger().info(f'Lifecycle publisher is active. Publishing: [{msg.data}]')
        else:
            self.get_logger().info('Lifecycle publisher is inactive. Messages are not published.')

        # We independently from the current state call publish on the lifecycle
        # publisher.
        # Only if the publisher is in an active state, the message transfer is
        # enabled and the message actually published.
        self._lifecycle_pub.publish(msg)

    ## Lifecycle callback for the configure state
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring...")

        # Your configure code here
        self._lifecycle_pub = self.create_lifecycle_publisher(String, 'lifecycle_publisher', 10)
        self._timer = self.create_timer(1.0, self.callback)

        self.get_logger().info("Node is configured!")

        return TransitionCallbackReturn.SUCCESS

    ##  Lifecycle callback for the activate state
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")

        # Your activation code here

        self.get_logger().info("Node is activated!")

        return super().on_activate(state)

    ##  Lifecycle callback for the deactivate state
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")

        # Your deactivation code here

        self.get_logger().info("Node is deactivated!")

        return super().on_deactivate(state)

    ##  Lifecycle callback for the cleanup state
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")

        # Your cleanup code here
        ## release resources
        self.destroy_timer(self._timer)
        self.destroy_publisher(self._lifecycle_pub)

        self.get_logger().info("Node is cleaned up!")

        return TransitionCallbackReturn.SUCCESS

    ##  Lifecycle callback for the shutdown state
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")

        # Your shutdown code here
        ## release resources
        self.destroy_timer(self._timer)
        self.destroy_publisher(self._lifecycle_pub)

        self.get_logger().info("Node is shut down!")

        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.SingleThreadedExecutor()
    lifecycle_node = MyLifecycleNode('lifecycle_node')
    executor.add_node(lifecycle_node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lifecycle_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
