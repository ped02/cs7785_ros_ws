#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
	public:
		MinimalSubscriber()
			: Node("minimal_subscriber")
		{
			constexpr size_t max_queue_length { 10 };
			m_subscription = this->create_subscription<std_msgs::msg::String>(
				"topic", max_queue_length, std::bind(&MinimalSubscriber::topic_callback, this, _1)
			);
		}

	private:
		void topic_callback(const std_msgs::msg::String& message) const
		{
			RCLCPP_INFO(this->get_logger(), "I heard: '%s'", message.data.c_str());
		}

		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscription;
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalSubscriber>());
	rclcpp::shutdown();
	return 0;
}
