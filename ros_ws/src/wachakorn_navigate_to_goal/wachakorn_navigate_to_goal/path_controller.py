import math
from numbers import Number
from typing import Tuple, Union

import numpy as np

import rclpy
from rclpy.node import Node, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rclpy.action import (
    ActionClient,
    GoalResponse,
    CancelResponse,
    ActionServer,
)
from rclpy.action.server import ServerGoalHandle

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid

from robot_interfaces.action import PathPlan, GoToTarget, Control

from wachakorn_chase_object.ros_utils import (
    path_to_np_points,
    parse_occupancy_grid,
    quat_message_to_rotation,
)

from .rrt import RRTStar


def signed_angle_difference(vector_a, vector_b):
    """Vector b is target vector."""
    x = np.sum(np.multiply(vector_a, vector_b))
    y = np.cross(vector_a, vector_b)
    return np.arctan2(y, x)


def rotation_2d(angle_radians):
    cos = np.cos(angle_radians)
    sin = np.sin(angle_radians)
    return np.array([[cos, -sin], [sin, cos]])


class PathController(Node):
    """Call path plan on next goal"""

    def __init__(self, **kwargs):
        super(PathController, self).__init__('path_controller_node', **kwargs)

        # Parameters
        self.map_blocked_threashold = 0.75

        self.angle_deviation_threshold_radians = 10 / 180 * math.pi
        self.rotate_robot_timeout_sec = 2.0

        self.distance_deviation_threshold_m = 0.2
        self.move_robot_timeout_sec = 2.0

        # Frames
        self.robot_frame_id = 'localize_odom'

        # Transform
        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer, self)

        # Map Subscriber
        self.state_subscriber_callback = MutuallyExclusiveCallbackGroup()

        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map/grid',
            self.set_map,
            3,
            callback_group=self.state_subscriber_callback,
        )
        self.reset_map()

        # State Subscriber
        self.obstacle_too_close_subscriber = self.create_subscription(
            Bool,
            '/obstacle_too_close',
            self.set_too_close,
            3,
            callback_group=self.state_subscriber_callback,
        )
        self.reset_too_close()

        # Go to Goal Action
        self.go_to_go_action_callback_group = MutuallyExclusiveCallbackGroup()
        self.action_server = ActionServer(
            self,
            GoToTarget,
            'go_to_target',
            execute_callback=self.go_execute_action_callback,
            callback_group=self.go_to_go_action_callback_group,
            goal_callback=self.go_goal_callback,
            cancel_callback=self.go_cancel_callback,
        )
        self.running = False

        # Feedback Displayer
        self.feedback_displayer_callback_group = (
            MutuallyExclusiveCallbackGroup()
        )
        self.feedback_output = self.create_publisher(
            CompressedImage,
            '/plan_controller/feedback/image/compressed',
            3,
            callback_group=self.feedback_displayer_callback_group,
        )

        self.rotation_error_output = self.create_publisher(
            Float64,
            '/plan_controller/feedback/rotation_error',
            3,
            callback_group=self.feedback_displayer_callback_group,
        )

        self.move_error_output = self.create_publisher(
            Float64,
            '/plan_controller/feedback/move_error',
            3,
            callback_group=self.feedback_displayer_callback_group,
        )

        # Actions
        self.plan_action_callback_group = MutuallyExclusiveCallbackGroup()
        self.planner_action_cient = ActionClient(
            self,
            PathPlan,
            '/path_plan',
            callback_group=self.plan_action_callback_group,
        )
        self.planning = False
        self.reset_path()

        self.rotate_action_callback_group = MutuallyExclusiveCallbackGroup()
        self.rotate_action_client = ActionClient(
            self,
            Control,
            '/rotate_robot',
            callback_group=self.rotate_action_callback_group,
        )

        self.linear_action_callback_group = MutuallyExclusiveCallbackGroup()
        self.move_action_client = ActionClient(
            self,
            Control,
            '/move_robot',
            callback_group=self.linear_action_callback_group,
        )

        # Setup rates
        self.rotate_action_sleep_rate = self.create_rate(25)
        self.move_sleep_rate = self.create_rate(25)

        self.execute_sleep_rate = self.create_rate(10)
        self.blocked_sleep_rate = self.create_rate(0.7)

        # Wait for action server
        while not self.planner_action_cient.wait_for_server(0.5):
            self.get_logger().info('Waiting for planner server')

        while not self.rotate_action_client.wait_for_server(0.5):
            self.get_logger().info('Waiting for rotate action server')

        while not self.move_action_client.wait_for_server(0.5):
            self.get_logger().info('Waiting for move action server')

    def reset_map(self):
        self.occupancy_grid_message = None

    def set_map(self, message: OccupancyGrid):
        self.occupancy_grid_message = message

    def reset_path(self):
        self.path_frame = None
        self.path = np.zeros((0, 2))

    def reset_too_close(self):
        self.too_close = False

    def set_too_close(self, message: Bool):
        self.too_close = message.data

    def get_current_pose(self) -> Union[Tuple[Number, Number, Number], None]:
        target_frame_id = self.occupancy_grid_message.header.frame_id
        robot_source_frame_id = self.robot_frame_id

        robot_transform = None
        try:
            robot_transform = self.transform_buffer.lookup_transform(
                target_frame_id,
                robot_source_frame_id,
                self.get_clock().now().to_msg(),
                # self.scan_timestamp.to_msg(),
                # (self.scan_timestamp - rclpy.duration.Duration(nanoseconds=50000000)).to_msg(),
                # timeout=rclpy.duration.Duration(nanoseconds=25000000)
                timeout=rclpy.duration.Duration(nanoseconds=75000000),
            )
        except TransformException as ex:
            self.get_logger().info(
                f"[Get Current Position] Could not transform '{robot_source_frame_id}' to '{target_frame_id}': {ex}"
            )

        if robot_transform is None:
            return None

        robot_rotation = quat_message_to_rotation(
            robot_transform.transform.rotation
        )
        z_rotation = robot_rotation.as_euler('xyz')[2]

        return [
            robot_transform.transform.translation.x,
            robot_transform.transform.translation.y,
            z_rotation,
        ]

    def plan(self, goal_position: Tuple[Number, Number]):
        """
        goal_position:
            goal position in map frame
        """

        if self.occupancy_grid_message is None:
            self.get_logger().info('No map')
            return

        robot_pose_tuple = self.get_current_pose()

        if robot_pose_tuple is None:
            self.get_logger().info('No start pose')
            return

        action_message = PathPlan.Goal()
        action_message.map = self.occupancy_grid_message
        action_message.start_pose = Pose2D(
            x=robot_pose_tuple[0], y=robot_pose_tuple[1], theta=0.0
        )
        action_message.goal_pose = Pose2D(
            x=goal_position[0], y=goal_position[1], theta=0.0
        )

        self.reset_path()
        self.planning = True

        self.plan_goal_handle = None
        self.goal_future = self.planner_action_cient.send_goal_async(
            action_message, feedback_callback=self.plan_feedback_callback
        )
        self.goal_future.add_done_callback(self.plan_response_callback)

    def plan_response_callback(self, future):
        self.plan_goal_handle = future.result()
        self.planning = self.plan_goal_handle.accepted

        if not self.plan_goal_handle.accepted:
            self.get_logger().info('Planning Rejected')
            return

        self.get_logger().info('Planing Accepted')

        self.get_result_future = self.plan_goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.plan_result_callback)

    def plan_feedback_callback(self, feedback_message):
        feedback: PathPlan.Feedback = feedback_message.feedback
        # self.get_logger().info(f'[Feedback] Nodes: {feedback.num_nodes}')
        self.feedback_output.publish(feedback.plan_debug)

    def plan_result_callback(self, future):
        result: PathPlan.Result = future.result().result

        self.path_frame = result.path.header
        self.path = path_to_np_points(result.path)
        self.planning = False

        self.get_logger().info(f'Finished Planning Path: {self.path}')

    def rotate_robot(self, target_angle_radians, timeout_sec=5.0) -> bool:
        """
        Parameters
        ----------
        target_angle_radians: float
            Target angle to rotate to relative to current.
        Returns
        -------
        success: bool
            if rotated within timeout
        """

        rotation_control_goal = Control.Goal()
        rotation_control_goal.target_value = float(target_angle_radians)

        self.get_logger().info('[Rotate Robot Action] Send goal')
        rotate_control_future = self.rotate_action_client.send_goal_async(
            rotation_control_goal
        )

        while not rotate_control_future.done():
            self.rotate_action_sleep_rate.sleep()

        rotate_goal_handle = rotate_control_future.result()

        rotate_result_future = rotate_goal_handle.get_result_async()

        control_start_time = self.get_clock().now()
        while True:
            control_time_elapsed_sec = (
                self.get_clock().now() - control_start_time
            ).nanoseconds * 1e-9

            if (
                timeout_sec is not None
                and control_time_elapsed_sec > timeout_sec
            ):
                # Timed out
                self.get_logger().info('[Rotate Robot Action] Timeout')
                break

            if rotate_result_future.done():
                break

            if self.too_close:
                # Cancel
                self.get_logger().info('[Rotate Robot Action] Too Close')
                # break

            self.rotate_action_sleep_rate.sleep()

        if not rotate_result_future.done():
            self.get_logger().info('[Rotate Robot Action] Cancelled')
            self.rotate_action_client._cancel_goal(rotate_goal_handle)

        return rotate_control_future.done()

    def move_robot(self, target_distance_m, timeout_sec=5.0) -> bool:
        """
        Parameters
        ----------
        target_distance_m: float
            Target distance to move to relative to current.
        Returns
        -------
        success: bool
            if rotated within timeout
        """

        move_control_goal = Control.Goal()
        move_control_goal.target_value = float(target_distance_m)
        self.get_logger().info('[Move Robot Action] Send goal')
        move_control_future = self.move_action_client.send_goal_async(
            move_control_goal
        )

        too_close = False

        while not move_control_future.done():
            self.move_sleep_rate.sleep()

        move_goal_handle = move_control_future.result()

        self.get_logger().info(f'[Move Robot Action] {move_goal_handle}')

        move_result_future = move_goal_handle.get_result_async()

        control_start_time = self.get_clock().now()

        done = False

        while True:
            control_time_elapsed_sec = (
                self.get_clock().now() - control_start_time
            ).nanoseconds * 1e-9

            if (
                timeout_sec is not None
                and control_time_elapsed_sec > timeout_sec
            ):
                # Timed out
                self.get_logger().info('[Move Robot Action] Timeout')
                break

            if move_result_future.done():
                self.get_logger().info('[Move Robot Action] Done')
                done = True
                break

            if self.too_close:
                # Cancel
                self.get_logger().info('[Move Robot Action] Too Close')
                too_close = True
                break

            self.move_sleep_rate.sleep()

        if not done:
            self.get_logger().warn('[Move Robot Action] Cancel')
            self.move_action_client._cancel_goal(move_goal_handle)
            self.get_logger().warn('[Move Robot Action] Cancelled')

        # return move_control_future.done()
        return too_close

    def get_current_map(self) -> RRTStar:
        map_grid, map_x_bounds, map_y_bounds = parse_occupancy_grid(
            self.occupancy_grid_message
        )

        # print(f'Map: low [{map_x_bounds[0]}, {map_y_bounds[0]}] High [{map_x_bounds[-1]}, {map_y_bounds[-1]}]')

        map_grid_gray = (map_grid.astype(np.float32)) / 100.0
        map_grid_binary = map_grid_gray >= self.map_blocked_threashold

        return RRTStar(map_grid_binary, map_x_bounds, map_y_bounds)

    def go_goal_callback(self, goal_handle):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')

        if self.running:
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def go_cancel_callback(self, cancel_request):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')

        if not self.running:
            return CancelResponse.REJECT

        # Stop planning
        self.running = False

        if self.plan_goal_handle is not None:
            self.planner_action_cient._cancel_goal(self.plan_goal_handle)

    def go_execute_action_callback(self, goal_handle: ServerGoalHandle):
        forward_local_vector = np.array([[1.0, 0.0]])

        goal_request: GoToTarget.Goal = goal_handle.request
        goal_pose = goal_request.goal_pose
        goal_pose_tuple = (goal_pose.x, goal_pose.y)
        goal_pose_np = np.array(goal_pose_tuple).reshape(1, 2)

        self.get_logger().info(f'Executing goal: {goal_pose_tuple=}')
        feedback_message = GoToTarget.Feedback()

        self.running = True
        reached_target = False

        wait_print_period_sec = 3
        period_printed = False

        while self.running and not reached_target:
            # Plan (Assume oath in same frame)
            self.get_logger().info('Starting plan')
            self.plan(goal_pose_tuple)
            plan_start_time = self.get_clock().now()

            period_printed = False
            while self.planning:
                # wait for path
                wait_duration_sec = (
                    self.get_clock().now() - plan_start_time
                ).nanoseconds * 1e-9
                if int(wait_duration_sec) % wait_print_period_sec == 0:
                    period_printed = True
                    if not period_printed:
                        self.get_logger().info(
                            f'Waiting for plan for {wait_duration_sec} seconds'
                        )
                else:
                    period_printed = False

                self.execute_sleep_rate.sleep()

            if len(self.path) == 0:
                # Re-try plan
                self.get_logger().info('Plan Failed')
                continue

            self.path = np.concatenate([self.path[1:], goal_pose_np], axis=0)

            # Put plan in map, create path mask and get index. Use to check for collision
            map_check = self.get_current_map()
            try:
                path_mask_indices = map_check.get_path_mask_indices(
                    self.path, inflate_radius=2
                )
            except ValueError:
                continue

            if map_check.check_index_colide(path_mask_indices):
                # Collided - Replan
                continue

            # Execute Plan
            executing_plan = True
            next_path_target_index = 0
            while executing_plan:
                too_close = False

                target_position = np.expand_dims(
                    self.path[next_path_target_index], 0
                )  # Assume path point in same frame as target

                # Send next_point to movement action
                current_position = self.get_current_pose()
                if current_position is None:
                    continue
                current_position_np = np.array(current_position).reshape(1, -1)
                forward_vector = (
                    forward_local_vector
                    @ rotation_2d(current_position_np[0, 2]).T
                )

                feedback_message.current_pose = Pose2D(
                    x=current_position_np[0, 0],
                    y=current_position_np[0, 1],
                    theta=current_position_np[0, 2],
                )
                goal_handle.publish_feedback(feedback_message)

                direction = target_position - current_position_np[:, :-1]

                angle_deviation_1 = signed_angle_difference(
                    forward_vector, direction
                )
                angle_deviation_2 = signed_angle_difference(
                    forward_vector, -direction
                )
                angle_deviation = min(
                    angle_deviation_1, angle_deviation_2, key=abs
                )
                self.get_logger().info(f'{angle_deviation=}')

                self.rotation_error_output.publish(
                    Float64(data=float(angle_deviation))
                )

                if (
                    abs(angle_deviation)
                    > self.angle_deviation_threshold_radians
                ):
                    # Rotate to match in steps
                    self.rotate_robot(
                        angle_deviation,
                        timeout_sec=self.rotate_robot_timeout_sec,
                    )

                # Revaluate plan
                map_check = self.get_current_map()
                if map_check.check_index_colide(path_mask_indices):
                    # Replan
                    break

                current_position = self.get_current_pose()
                if current_position is None:
                    continue
                current_position_np = np.array(current_position).reshape(1, -1)
                forward_vector = (
                    forward_local_vector
                    @ rotation_2d(current_position_np[0, 2]).T
                )

                print(f'[Feedback] {current_position=}')
                feedback_message.current_pose = Pose2D(
                    x=current_position_np[0, 0],
                    y=current_position_np[0, 1],
                    theta=current_position_np[0, 2],
                )
                goal_handle.publish_feedback(feedback_message)

                direction = target_position - current_position_np[:, :-1]

                distance_deviation = np.sum(
                    np.multiply(direction, forward_vector)
                )

                self.get_logger().info(f'{distance_deviation=}')
                self.move_error_output.publish(
                    Float64(data=float(distance_deviation))
                )

                distance_threshold = (
                    self.distance_deviation_threshold_m
                    if next_path_target_index < len(self.path) - 1
                    else goal_request.deviation
                )

                if abs(distance_deviation) > distance_threshold:
                    # Move to match in steps
                    too_close = self.move_robot(
                        distance_deviation,
                        timeout_sec=self.move_robot_timeout_sec,
                    )

                if too_close:
                    # Replan
                    self.get_logger().info('Too close move. Replan')
                    self.blocked_sleep_rate.sleep()
                    break

                map_check = self.get_current_map()
                if map_check.check_index_colide(path_mask_indices):
                    # Replan
                    break

                current_position = self.get_current_pose()
                if current_position is None:
                    continue
                current_position_np = np.array(current_position).reshape(1, -1)

                feedback_message.current_pose = Pose2D(
                    x=current_position_np[0, 0],
                    y=current_position_np[0, 1],
                    theta=current_position_np[0, 2],
                )
                goal_handle.publish_feedback(feedback_message)

                direction = target_position - current_position_np[:, :-1]
                distance_deviation = np.linalg.norm(direction)
                if distance_deviation <= distance_threshold:
                    # Reached current point
                    next_path_target_index += 1

                    if len(self.path) == next_path_target_index:
                        reached_target = True
                        executing_plan = False
                        break

        result_message = GoToTarget.Result()

        current_position = self.get_current_pose()
        if current_position is not None:
            current_position_np = np.array(current_position).reshape(1, -1)

            direction = target_position - current_position_np[:, :-1]
            distance_deviation = np.squeeze(np.linalg.norm(direction))

            within_goal = distance_deviation <= goal_request.deviation

            result_message.final_pose = Pose2D(
                x=current_position_np[0, 0],
                y=current_position_np[0, 1],
                theta=current_position_np[0, 2],
            )
            result_message.reached = bool(within_goal)

        self.running = False

        self.get_logger().info('Action succeeded')

        goal_handle.succeed()

        return result_message


def main(args=None):
    rclpy.init(args=args)

    path_controller = PathController()

    executor = MultiThreadedExecutor()
    executor.add_node(path_controller)

    # Spin Node
    executor.spin()

    # Destroy node
    path_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
