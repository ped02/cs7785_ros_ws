import time

import rclpy
from rclpy.node import Node, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rclpy.action import ActionClient

from geometry_msgs.msg import Pose2D

from robot_interfaces.action import GoToTarget


class TargetCaller(Node):
    def __init__(self, **kwargs):
        super(TargetCaller, self).__init__('target_caller_node', **kwargs)

        self.targets = [(1.42, 0.0, 0.02), (1.4, 1.5, 0.1), (0.0, 1.5, 0.1)]
        # self.target_deviation_m = 0.1

        action_callback_group = MutuallyExclusiveCallbackGroup()
        self.go_to_target_action_client = ActionClient(
            self,
            GoToTarget,
            '/go_to_target',
            callback_group=action_callback_group,
        )

        self.called = False

        timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.call_timer = self.create_timer(
            5, self.call_targets, callback_group=timer_callback_group
        )

        self.loop_sleep_rate = self.create_rate(25)
        self.between_point_sleep_rate = self.create_rate(frequency=0.25)

    def call_targets(self):
        if self.called:
            return

        self.called = True

        target_count = len(self.targets)
        target_id = 0

        goal_handle = None

        while target_id < target_count:
            self.get_logger().info(f'Starting goal {target_id}')
            current_target = self.targets[target_id]

            go_to_target_goal = GoToTarget.Goal()
            go_to_target_goal.goal_pose = Pose2D(
                x=float(current_target[0]), y=float(current_target[1])
            )
            go_to_target_goal.deviation = float(current_target[2])

            go_to_target_future = (
                self.go_to_target_action_client.send_goal_async(
                    go_to_target_goal
                )
            )

            while not go_to_target_future.done():
                # self.get_logger().info('A')
                self.loop_sleep_rate.sleep()
                time.sleep(0.01)

            # self.get_logger().info('B')
            goal_handle = go_to_target_future.result()
            # goal_handle = self.go_to_target_action_client.send_goal(go_to_target_goal)
            # self.get_logger().info('C')
            go_to_target_result_future = goal_handle.get_result_async()
            # self.get_logger().info('D')

            while True:
                # self.get_logger().info('E')
                if go_to_target_result_future.done():
                    break

                self.loop_sleep_rate.sleep()
                time.sleep(0.01)

            if not go_to_target_result_future.done():
                # self.go_to_target_action_client._cancel_goal(goal_handle)
                pass

            # go_to_target_result = go_to_target_result_future.result

            # status = go_to_target_result.status

            # self.get_logger().info(f'Status: {status}')
            # if status == GoalStatus.STATUS_SUCCEEDED:
            #     # Success
            goal_handle = None
            target_id += 1

            self.get_logger().info('Sleeping')
            self.between_point_sleep_rate.sleep()
            time.sleep(4)
            self.get_logger().info('Slept')

        if goal_handle is not None:
            # Cancel
            self.get_logger().info('Cancelling Target')
            # self.go_to_target_action_client._cancel_goal(goal_handle)
            pass


def main(args=None):
    rclpy.init(args=args)

    target_caller_node = TargetCaller()

    executor = MultiThreadedExecutor()
    executor.add_node(target_caller_node)
    # target_caller_node.call_targets()

    # Spin Node
    # rclpy.spin(target_caller_node)
    executor.spin()

    # Destroy node
    target_caller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
