from numbers import Number
from typing import Tuple

import numpy as np

import cv2 as cv

import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path

from cv_bridge import CvBridge

from robot_interfaces.action import PathPlan

from wachakorn_chase_object.ros_utils import parse_occupancy_grid
from .rrt import RRTStar


class PathPlannerActionServer(Node):
    def __init__(self, **kwargs):
        super(PathPlannerActionServer, self).__init__(
            'path_planner_action_server', **kwargs
        )

        # Map
        self.map_blocked_threashold = 0.75
        self.map_dilate_radius_px = 6  # 2 # ~ robot radius # Lab 4
        # self.map_dilate_radius_px = 20 #2 # ~ robot radius
        self.map_border_px = 2

        # Search parameters
        self.steer_size = 0.3
        self.greedy_bias = 0.1
        self.neighbour_radius = 3.0 * self.steer_size  # 2.5 * self.steer_size

        self.max_iteration = 10000
        self.max_nodes = 10000

        # Feed back
        self.feedback_frame_skip = 5

        self.action_server = ActionServer(
            self,
            PathPlan,
            'path_plan',
            execute_callback=self.execute_action_callback,
            # callback_group=self.action_callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.running = False

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')

        if self.running:
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, cancel_request):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')

        if not self.running:
            return CancelResponse.REJECT

        # Stop planning
        self.running = False

        return CancelResponse.ACCEPT

    def execute_action_callback(self, goal_handle: ServerGoalHandle):
        self.running = True

        goal_request: PathPlan.Goal = goal_handle.request

        map_message = goal_request.map
        start_pose_message = goal_request.start_pose
        goal_pose_message = goal_request.goal_pose

        feedback_message = PathPlan.Feedback()

        result_message = PathPlan.Result()

        start_position_tuple = (start_pose_message.x, start_pose_message.y)
        # start_position = np.array(
        #     [[start_pose_message.x, start_pose_message.y]]
        # )
        # start_theta = start_pose_message.theta

        goal_position_tuple = (goal_pose_message.x, goal_pose_message.y)
        # goal_position = np.array([[goal_pose_message.x, goal_pose_message.y]])
        # goal_theta = goal_pose_message.theta

        map_grid, map_x_bounds, map_y_bounds = parse_occupancy_grid(map_message)

        print(
            f'Map: low [{map_x_bounds[0]}, {map_y_bounds[0]}] High [{map_x_bounds[-1]}, {map_y_bounds[-1]}]'
        )

        map_grid_gray = (map_grid.astype(np.float32)) / 100.0
        map_grid_binary = map_grid_gray >= self.map_blocked_threashold
        map_grid_binary_original = map_grid_binary.copy()

        if self.map_dilate_radius_px != 0:
            kernel_size = 2 * (self.map_dilate_radius_px // 2) + 1

            expand_element = cv.getStructuringElement(
                cv.MORPH_ELLIPSE, (kernel_size, kernel_size)
            )
            map_grid_binary = cv.morphologyEx(
                map_grid_binary.astype(np.uint8),
                cv.MORPH_DILATE,
                expand_element,
            ).astype(bool)

        if self.map_border_px != 0:
            map_grid_binary[: self.map_border_px] = True
            map_grid_binary[-self.map_border_px :] = True
            map_grid_binary[:, : self.map_border_px] = True
            map_grid_binary[:, -self.map_border_px :] = True

        target_frame_id = map_message.header.frame_id

        rrt_non_inflat = RRTStar(
            map_grid_binary_original, map_x_bounds, map_y_bounds
        )

        set_goal_success_non_inflate = rrt_non_inflat.set_goals(
            start_position_tuple, goal_position_tuple
        )

        if not set_goal_success_non_inflate:
            # Robot inside obstacle
            goal_handle.abort()
            self.running = False
            return result_message

        # Inflation target

        rrt = RRTStar(map_grid_binary, map_x_bounds, map_y_bounds)

        set_goal_success = rrt.set_goals(
            start_position_tuple, goal_position_tuple
        )

        # if not set_goal_success:
        #     # Inflation messed with planning, need to clear around robot

        #     # Find clostest non obstcle block
        def get_clear_inflate_indices(position: Tuple[Number, Number]):
            nonlocal rrt
            position_np = np.array(position).reshape(1, 2)

            block_centers_flat = rrt.map_centers.reshape(-1, 2)
            if rrt.is_position_blocked(position):
                # Find closest non block in inflate map
                position_to_block_centers_distances = rrt.get_centers_distance(
                    position_np
                )

                self.get_logger().info(
                    f'{position_to_block_centers_distances.shape=}'
                )

                position_to_block_centers_distances_flat = (
                    position_to_block_centers_distances.reshape(-1)
                )
                position_to_non_blocked_centers_flat_indices = np.where(
                    map_grid_binary.reshape(-1)
                )[0]

                self.get_logger().info(
                    f'{position_to_block_centers_distances_flat.shape=}'
                )
                self.get_logger().info(f'{map_grid_binary.reshape(-1).shape=}')

                position_to_non_blocked_centers_distances_flat = (
                    position_to_block_centers_distances_flat[
                        position_to_non_blocked_centers_flat_indices
                    ]
                )
                block_centers_non_blocked_flat = block_centers_flat[
                    position_to_non_blocked_centers_flat_indices
                ]

                position_to_non_blocked_blocks_sorted_indices = np.argsort(
                    position_to_non_blocked_centers_distances_flat
                )

                for i in position_to_non_blocked_blocks_sorted_indices:
                    # Start to i
                    path_grid_indices = rrt_non_inflat.get_line_grid_indices(
                        position_np,
                        np.expand_dims(block_centers_non_blocked_flat[i], 0),
                        inflate_radius=1.5,
                    )

                    path_blocks_values = rrt_non_inflat.map[
                        path_grid_indices[:, 1], path_grid_indices[:, 0]
                    ]

                    if not np.any(path_blocks_values):
                        return path_grid_indices

        if not set_goal_success:
            self.get_logger().info('Relaxing point')
            if rrt.is_position_blocked(start_position_tuple):
                start_clear_indices = get_clear_inflate_indices(
                    start_position_tuple
                )
                map_grid_binary[
                    start_clear_indices[:, 1], start_clear_indices[:, 0]
                ] = False

            if rrt.is_position_blocked(goal_position_tuple):
                goal_clear_indices = get_clear_inflate_indices(
                    goal_position_tuple
                )
                map_grid_binary[
                    goal_clear_indices[:, 1], goal_clear_indices[:, 0]
                ] = False

        rrt = RRTStar(map_grid_binary, map_x_bounds, map_y_bounds)

        set_goal_success = rrt.set_goals(
            start_position_tuple, goal_position_tuple
        )

        if not set_goal_success:
            goal_handle.abort()
            return result_message

        print(f'RRT Start: {rrt.start_position} Goal: {rrt.goal_position}')
        print(
            f'RRT Index Start: {rrt._get_grid_index(rrt.start_position_np)} Goal: {rrt._get_grid_index(rrt.goal_position_np)}'
        )

        # TODO: Check if blocked

        def get_progress_image():
            nonlocal rrt
            fig = plt.figure(0)
            ax = fig.gca()

            rrt.plot(ax)
            ax.legend()
            ax.grid()
            ax.invert_yaxis()

            fig.canvas.draw()
            image_plot = np.array(fig.canvas.renderer.buffer_rgba())[..., :-1]
            plt.close(fig)

            return image_plot

        # bridge = CvBridge()

        current_frame = 0
        found = False
        last_show_node = -1
        while self.running and not found:
            # Running

            found = rrt.step(
                dQ=self.steer_size,
                greedy_bias=self.greedy_bias,
                neighbour_radius=self.neighbour_radius,
            )

            if self.feedback_frame_skip != 0:
                if current_frame % self.feedback_frame_skip == 0:
                    # Publish Feedback
                    if last_show_node != len(rrt.nodes):
                        feedback_message.plan_debug = (
                            CvBridge().cv2_to_compressed_imgmsg(
                                get_progress_image()
                            )
                        )
                        feedback_message.num_nodes = len(rrt.nodes)

                        goal_handle.publish_feedback(feedback_message)
                        last_show_node = len(rrt.nodes)
                        # self.get_logger().info('Publishing feedback')

            current_frame += 1

            if current_frame >= self.max_iteration:
                break

            if len(rrt.nodes) >= self.max_nodes:
                break

        if found:
            # Trace path
            path = rrt.trace_path()

            pose_header = Header(
                frame_id=target_frame_id, stamp=self.get_clock().now().to_msg()
            )

            path_pose = [
                PoseStamped(
                    header=pose_header,
                    pose=Pose(
                        position=Point(
                            x=p[0],
                            y=p[1],
                        )
                    ),
                )
                for p in path
            ]

            result_message.path = Path(header=pose_header, poses=path_pose)

        goal_handle.succeed()
        self.running = False

        del rrt

        return result_message


def main(args=None):
    rclpy.init(args=args)

    path_plan_action_server = PathPlannerActionServer()

    # Spin Node
    rclpy.spin(path_plan_action_server)

    # Destroy node
    path_plan_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
