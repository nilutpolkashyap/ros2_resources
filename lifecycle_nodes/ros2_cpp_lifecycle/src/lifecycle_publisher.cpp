#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <memory>

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode 
{
  	public:
		//MyLifecycleNode constructor
		explicit MyLifecycleNode(const std::string &nodeName, bool intraProcessComms = false)
			: rclcpp_lifecycle::LifecycleNode(nodeName,
					rclcpp::NodeOptions().use_intra_process_comms(intraProcessComms)) {}

		void publish_callback() {
			static size_t count = 0;
			auto msg = std::make_unique<std_msgs::msg::String>();
			msg->data = "Lifecycle Hello World -> " + std::to_string(++count);

			if (!_lifecycle_pub->is_activated()) {
			RCLCPP_INFO(get_logger(), "Lifecycle publisher is inactive. Messages are not published.");
			} else {
			RCLCPP_INFO(get_logger(), "Lifecycle publisher is active. Publishing: [%s]",
						msg->data.c_str());
			}

			_lifecycle_pub->publish(std::move(msg));
		}

		CallbackReturn on_configure(const rclcpp_lifecycle::State &) 
		{
			RCLCPP_INFO(get_logger(), "Configuring...");
			
			// Your configure code here
			_lifecycle_pub = this->create_publisher<std_msgs::msg::String>("lifecycle_topic", 10);
			_timer = this->create_wall_timer(1s, std::bind(&MyLifecycleNode::publish_callback, this));

			RCLCPP_INFO(get_logger(), "Node is configured!");

			return CallbackReturn::SUCCESS;
		}

		CallbackReturn on_activate(const rclcpp_lifecycle::State & state) 
		{
			RCLCPP_INFO(get_logger(), "Activating...");

        	// Your activation code here
			// _lifecycle_pub->on_activate();
			LifecycleNode::on_activate(state);
			std::this_thread::sleep_for(2s);

			RCLCPP_INFO(get_logger(), "Node is activated!");

			return CallbackReturn::SUCCESS;
		}

		CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) 
		{
			RCLCPP_INFO(get_logger(), "Deactivating...");

        	// Your deactivation code here
			// publisher->on_deactivate();
			LifecycleNode::on_deactivate(state);

        	RCLCPP_INFO(get_logger(), "Node is deactivated!");

			return CallbackReturn::SUCCESS;
		}

		CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) 
		{
			RCLCPP_INFO(get_logger(), "Cleaning up...");

			// Your cleanup code here
			_lifecycle_pub.reset();
			_timer.reset();
			
			RCLCPP_INFO(get_logger(), "Node is cleaned up!");

			return CallbackReturn::SUCCESS;
		}

		CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) 
		{
			RCLCPP_INFO(get_logger(), "Shutting down...");

			// Your shutdown code here
			_lifecycle_pub.reset();
			_timer.reset();

			RCLCPP_INFO(get_logger(), "Node is shut down!");

			return CallbackReturn::SUCCESS;
		}

	private:
		std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> _lifecycle_pub;
		std::shared_ptr<rclcpp::TimerBase> _timer;
};

int main(int argc, char *argv[]) {
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor executor;
	auto lifecycle_node = std::make_shared<MyLifecycleNode>("lifecycle_node");
	executor.add_node(lifecycle_node->get_node_base_interface());

	try {
		executor.spin();
	} 
	catch (const std::exception &e) {
		RCLCPP_ERROR(lifecycle_node->get_logger(), "Unhandled exception: %s", e.what());
	}

	rclcpp::shutdown();

  	return 0;
}