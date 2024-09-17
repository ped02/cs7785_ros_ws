import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist, Vector3


# TODO: Refactor using ROS2 parameter
class RobotRotator(Node):
    def __init__(self):
        super(RobotRotator, self).__init__('robot_rotator')

        self.subscription = self.create_subscription(
            Point, '/image_point', self.command_rotation, 10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 5)

        self.timer_period_sec = 0.5
        self.stop_time_sec = 2
        self.last_receive_time = time.time()

        self.stop_timer = self.create_timer(
            self.timer_period_sec, self.command_stop
        )

    def command_rotation(self, object_point: Point):
        DEADZONE = 30

        IMAGE_WIDTH = 320

        IMAGE_CENTER = IMAGE_WIDTH // 2

        deviations = [DEADZONE, 70, 130, IMAGE_WIDTH // 2 + 1]
        angular_rates = [0.0, 0.25, 0.5, 0.75]

        for deviation, angular_rate in zip(deviations, angular_rates):
            error = object_point.x - IMAGE_CENTER
            if abs(error) < deviation:
                rotate_command = Twist(
                    angular=Vector3(z=-(error / abs(error)) * angular_rate)
                )
                self.publisher.publish(rotate_command)
                self.last_receive_time = time.time()
                break

    def command_stop(self):
        # self.get_logger().info(f'Ticking')
        if time.time() - self.last_receive_time > self.stop_time_sec:
            # Stop
            rotate_command = Twist(angular=Vector3(z=0.0))
            self.publisher.publish(rotate_command)


def main(args=None):
    rclpy.init(args=args)

    robot_rotator = RobotRotator()

    rclpy.spin(robot_rotator)

    robot_rotator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
