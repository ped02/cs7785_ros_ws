#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
	public:
		MinimalPublisher()
			: Node("minimal_publisher")
			, m_count { 0 }
		{
			constexpr size_t max_queue_length { 10 };
			m_publisher = this->create_publisher<std_msgs::msg::String>("topic", max_queue_length);
			m_timer = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
		}

	private:
		void timer_callback()
		{
			auto message = std_msgs::msg::String();
			message.data = "Hello, world!" + std::to_string(m_count);
			++m_count;
			RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
			m_publisher->publish(message);
		}

		rclcpp::TimerBase::SharedPtr m_timer;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
		size_t m_count;
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalPublisher>());
	rclcpp::shutdown();
	return 0;
}
