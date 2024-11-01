import sys
import argparse
import time

import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient

from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid

from robot_interfaces.action import PathPlan


class PathPlanTester(Node):
    def __init__(self, **kwargs):
        super(PathPlanTester, self).__init__('path_plan_tester_node', **kwargs)

        # Map Subscriber
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, '/map/grid', self.set_map, 3
        )
        self.reset_map()

        # Feedback Displayer
        self.feedback_output = self.create_publisher(
            CompressedImage, '/plan_tester/feedback/image/compressed', 3
        )

        self.action_cient = ActionClient(self, PathPlan, '/path_plan')

        while not self.action_cient.wait_for_server(0.1):
            self.get_logger().info('Waiting for action server')

    def reset_map(self):
        self.occupancy_grid_message = None

    def set_map(self, message: OccupancyGrid):
        self.occupancy_grid_message = message

    def has_map(self) -> bool:
        return self.occupancy_grid_message is not None

    def call_map(self, start_position, end_position):
        action_message = PathPlan.Goal()
        action_message.map = self.occupancy_grid_message
        action_message.start_pose = Pose2D(
            x=start_position[0], y=start_position[1], theta=0.0
        )
        action_message.goal_pose = Pose2D(
            x=end_position[0], y=end_position[1], theta=0.0
        )

        self.goal_future = self.action_cient.send_goal_async(
            action_message, feedback_callback=self.goal_feedback_callback
        )
        self.goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def goal_feedback_callback(self, feedback_message):
        feedback: PathPlan.Feedback = feedback_message.feedback
        self.get_logger().info(f'[Feedback] Nodes: {feedback.num_nodes}')
        self.feedback_output.publish(feedback.plan_debug)
        # self.get_logger().info(f'Pub')

    def get_result_callback(self, future):
        result: PathPlan.Result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.path))

        sleep_rate = self.create_rate(1 / 2)
        sleep_rate.sleep()
        rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(description='Plan tester')
    parser.add_argument(
        '-sx', type=float, required=True, help='Start x coordinate'
    )
    parser.add_argument(
        '-sy', type=float, required=True, help='Start y coordinate'
    )
    parser.add_argument(
        '-gx', type=float, required=True, help='Goal x coordinate'
    )
    parser.add_argument(
        '-gy', type=float, required=True, help='Goal y coordinate'
    )

    rclpy.init(args=args)

    args_no_ros = rclpy.utilities.remove_ros_args(sys.argv)

    args_parsed = parser.parse_args(args=args_no_ros[1:])

    path_plan_tester = PathPlanTester()

    # sleep_rate = path_plan_tester.create_rate(2)

    rclpy.spin_once(path_plan_tester)
    while not path_plan_tester.has_map():
        path_plan_tester.get_logger().info('No map')
        rclpy.spin_once(path_plan_tester, timeout_sec=0.1)
        # sleep_rate.sleep()
        time.sleep(0.1)

    path_plan_tester.get_logger().info('Call map')
    path_plan_tester.call_map(
        (args_parsed.sx, args_parsed.sy), (args_parsed.gx, args_parsed.gy)
    )

    # Spin Node
    rclpy.spin(path_plan_tester)

    # Destroy node
    rclpy.shutdown()


if __name__ == '__main__':
    main()
