import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super(MinimalPublisher, self).__init__('minimal_publisher')
        max_queue_length = 10
        self.publisher = self.create_publisher(
            String, 'topic', max_queue_length
        )
        timer_period_sec = 0.5
        self.timer = self.create_timer(timer_period_sec, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        message = String()
        message.data = f'Hello World: {self.count}'
        self.count += 1
        self.publisher.publish(message)
        self.get_logger().info(f'Publishing: "{message.data}"')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Explicit for control (otherwise gc will destroy)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
