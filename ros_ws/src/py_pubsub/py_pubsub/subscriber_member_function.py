import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):
    def __init__(self):
        super(MinimalSubscriber, self).__init__('minimal_subscriber')
        max_queue_length = 10
        self.subscription = self.create_subscription(
            String, 'topic', self.listener_callback, max_queue_length
        )

    def listener_callback(self, message: String):
        self.get_logger().info(f'I heard: "{message.data}"')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
