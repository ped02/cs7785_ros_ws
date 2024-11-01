from typing import Tuple

import numpy as np

import rclpy
import rclpy.duration
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud

from wachakorn_chase_object.camera_utils import to_homogeneous
from wachakorn_chase_object.ros_utils import (
    point_cloud_to_np,
    quat_message_to_rotation,
    np_to_point_cloud,
)

from .geometry_utils import (
    get_point_line_parallel_perpendicular,
    line_vector_form_from_line_segment,
)


class MapperNode(Node):
    def __init__(self, **kwargs):
        super(MapperNode, self).__init__('mapper_node', **kwargs)

        # Parameter set at the start
        # Map for run (small)
        # self.map_x_low = -0.25
        # self.map_x_high = 2.25

        # self.map_y_low = -0.25
        # self.map_y_high = 2.25

        # self.map_division_count = 20

        # Map for testing (big)
        self.map_x_low = -2.0
        self.map_x_high = 3.0

        self.map_y_low = -2.0
        self.map_y_high = 3.0

        self.map_division_count = 100

        # Map for testing lab 5
        # self.map_x_low = -1.0
        # self.map_x_high = 3.0

        # self.map_y_low = -1.0
        # self.map_y_high = 3.0

        # self.map_division_count = 200

        self.reset_map()

        # Measurement Parameter
        # Axis 0 -> Detect or not Detect [0 - not blocked, 1 - detect blocked]
        # Axis 1 -> Hypothesize if free or not free [0 - free, 1 - not free]
        self.lidar_measurement_model = np.array([[0.6, 0.4], [0.2, 0.8]])
        # self.caution_bound = 0.2 # Lab 4
        self.caution_bound = 0.05  # Lab 4

        # Transform
        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer, self)

        # Pose Subscriber
        # self.odom_subscriber = self.create_subscription(Odometry, '/estimate_odom', self.set_odom, 10)

        point_cloud_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.reset_point_cloud()
        self.scan_point_subscriber = self.create_subscription(
            PointCloud,
            '/scan/point_cloud',
            self.set_point_cloud,
            point_cloud_qos_profile,
        )

        # Publisher
        self.map_publisher = self.create_publisher(
            OccupancyGrid, '/map/grid', 3
        )
        self.point_publisher = self.create_publisher(
            PointCloud, '/map/point_cloud', point_cloud_qos_profile
        )

        # Timer Process

        # PROCESS_TIMER_POLL_PERIOD_SEC = 0.02
        # self.process_timer = self.create_timer(
        #     PROCESS_TIMER_POLL_PERIOD_SEC,
        #     self.process_points,
        # )

        # self.processed = False

    def reset_map(self):
        self.map_x_range = self.map_x_high - self.map_x_low
        self.map_y_range = self.map_y_high - self.map_y_low

        self.map_x_cell_size = self.map_x_range / self.map_division_count
        self.map_y_cell_size = self.map_y_range / self.map_division_count

        self.map_corner_phi = np.arctan2(
            self.map_y_cell_size, self.map_x_cell_size
        )

        self.map_x_bounds = np.arange(
            self.map_x_low,
            self.map_x_high + (self.map_x_cell_size / 2),
            self.map_x_cell_size,
        )
        self.map_x_bounds_expanded = np.expand_dims(self.map_x_bounds, -1)

        self.map_y_bounds = np.arange(
            self.map_y_low,
            self.map_y_high + (self.map_y_cell_size / 2),
            self.map_y_cell_size,
        )
        self.map_y_bounds_expanded = np.expand_dims(self.map_y_bounds, -1)

        self.map_bounds = np.stack(
            np.meshgrid(self.map_x_bounds, self.map_y_bounds), -1
        )
        self.map_centers = self.map_bounds[:-1, :-1] + np.array(
            [[[self.map_x_cell_size / 2, self.map_y_cell_size / 2]]]
        )

        self.map = 0.5 * np.ones(
            (self.map_division_count, self.map_division_count), dtype=np.float64
        )

    def get_grid_index(
        self, points, return_partial_index=False
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Returns
        -------

        N x 2
        """

        # Take in points in local frame
        # Returns grid index + valid points

        x_slot = np.logical_and(
            self.map_x_bounds_expanded[:-1] < points[:, 0],
            points[:, 0] <= self.map_x_bounds_expanded[1:],
        )
        y_slot = np.logical_and(
            self.map_y_bounds_expanded[:-1] < points[:, 1],
            points[:, 1] <= self.map_y_bounds_expanded[1:],
        )

        point_valid_mask = np.logical_and(
            np.any(x_slot, axis=0), np.any(y_slot, axis=0)
        )

        grid_index = np.zeros_like(points, dtype=int)
        x_index, point_index_x = np.where(x_slot)
        grid_index[point_index_x, 0] = x_index

        y_index, point_index_y = np.where(y_slot)
        grid_index[point_index_y, 1] = y_index

        valid_indices = grid_index[point_valid_mask]

        if not return_partial_index:
            return valid_indices, point_valid_mask
        else:
            return valid_indices, point_valid_mask, grid_index

    def update_parameters(self):
        # TODO
        pass

    def reset_point_cloud(self):
        self.scan_frame_id = ''
        self.scan_timestamp = rclpy.time.Time(
            clock_type=self.get_clock().clock_type
        )

        self.scan_points = np.zeros(0)
        self.scan_intensities = np.zeros(0)

    def set_point_cloud(self, message: PointCloud):
        self.scan_frame_id = message.header.frame_id
        self.scan_timestamp = rclpy.time.Time.from_msg(message.header.stamp)

        # Convert to np array message.points
        self.scan_points, self.scan_intensities = point_cloud_to_np(message)

        self.process_points()

    def process_points(self):
        self.update_parameters()

        # self.get_logger().info('Process')
        # Transform points into plane frame

        target_frame_id = 'localize_base'
        point_source_frame_id = self.scan_frame_id
        # robot_source_frame_id = 'localize_odom'

        point_transform = None
        try:
            point_transform = self.transform_buffer.lookup_transform(
                target_frame_id,
                point_source_frame_id,
                # self.get_clock().now().to_msg(),
                # self.scan_timestamp.to_msg(),
                (
                    self.scan_timestamp
                    - rclpy.duration.Duration(nanoseconds=100000000)
                ).to_msg(),
                timeout=rclpy.duration.Duration(nanoseconds=75000000),
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform '{point_source_frame_id}' to '{target_frame_id}': {ex}"
            )

        if point_transform is None:
            # Fail: wait to try again
            return

        # self.get_logger().info(f'Source: {source_frame_id} Target: {target_frame_id}')

        point_rotation = quat_message_to_rotation(
            point_transform.transform.rotation
        )

        point_transform_matrix = np.eye(4, 4, dtype=np.float32)
        point_transform_matrix[:3, :3] = point_rotation.as_matrix()
        point_transform_matrix[:3, 3] = np.array(
            [
                point_transform.transform.translation.x,
                point_transform.transform.translation.y,
                point_transform.transform.translation.z,
            ]
        )

        if len(self.scan_points) == 0:
            self.get_logger().info('No Scan point')
            return

        scan_points = np.concatenate(
            [self.scan_points, np.zeros((len(self.scan_points), 1))], axis=-1
        )

        scan_points_homogeneous = to_homogeneous(scan_points)
        scan_points_homogeneous_transformed = (
            scan_points_homogeneous @ point_transform_matrix.T
        )

        scan_points_local = scan_points_homogeneous_transformed[:, :-1]
        scan_points_local_2d = scan_points_local[:, :-1]

        # Optimization Opportunity: Screen out invalid before -> less comparison overall

        # Get valid points and their indices
        (
            point_grid_indices,
            valid_mask,
            point_grid_partial_index,
        ) = self.get_grid_index(scan_points_local_2d, return_partial_index=True)

        # valid_points = scan_points_local_2d[valid_mask]

        collide_grid = np.zeros_like(self.map_centers[..., 0], dtype=bool)
        collide_grid[point_grid_indices[:, 1], point_grid_indices[:, 0]] = True

        # n_points = len(valid_points)

        robot_center = np.array([[0.0, 0.0, 0.0, 1.0]])
        robot_center_local_homogeneous = robot_center @ point_transform_matrix.T
        robot_center_local_2d = robot_center_local_homogeneous[:, :-2]

        # Get 'free'
        free_grid = np.zeros_like(self.map_centers[..., 0], dtype=bool)

        (
            robot_center_grid_index,
            robot_center_valid,
            robot_center_partial_index,
        ) = self.get_grid_index(
            robot_center_local_2d, return_partial_index=True
        )
        if not np.any(robot_center_valid):
            robot_center_grid_index = robot_center_partial_index.reshape(1, 2)
            if robot_center_local_2d[0, 0] > self.map_x_high:
                robot_center_grid_index[0, 0] = len(self.map[0]) - 1

            if robot_center_local_2d[0, 1] > self.map_y_high:
                robot_center_grid_index[0, 1] = len(self.map) - 1
        else:
            free_grid[
                robot_center_grid_index[0, 1], robot_center_grid_index[0, 0]
            ] = True

        # for i, scan_point in enumerate(valid_points):
        k = 0
        for i, scan_point in enumerate(scan_points_local_2d):
            if valid_mask[i]:
                grid_index_1 = point_grid_indices[k].reshape(-1, 2)
                k += 1
            else:
                grid_index_1 = point_grid_partial_index[i].reshape(1, 2)
                if scan_point[0] > self.map_x_high:
                    grid_index_1[0, 0] = len(self.map[0]) - 1

                if scan_point[1] > self.map_y_high:
                    grid_index_1[0, 1] = len(self.map) - 1

            grid_range = np.concatenate(
                [robot_center_grid_index, grid_index_1], axis=0
            )
            grid_range = np.sort(grid_range, axis=0)
            grid_range[1] += 1

            line_range_centers = self.map_centers[
                grid_range[0, 1] : grid_range[1, 1],
                grid_range[0, 0] : grid_range[1, 0],
            ]

            if line_range_centers.size == 0:
                continue

            line_range_centers_flat = line_range_centers.reshape(-1, 2)

            line_p, line_v_hat = line_vector_form_from_line_segment(
                robot_center_local_2d, np.expand_dims(scan_point, axis=0)
            )

            # No need to filter maybe? Consider line passed to be free
            # line_segment_end_parallel_distance, _ = get_point_line_parallel_perpendicular(np.concatenate([robot_center_local_2d, scan_point], axis=0), line_p, line_v_hat)

            # line_segment_end_bounds = np.sort(line_segment_end_parallel_distance,axis=0)

            line_perpendicular_angle = np.arctan2(
                abs(line_v_hat[0, 1]), abs(line_v_hat[0, 0])
            )

            if line_perpendicular_angle < self.map_corner_phi:
                # left - right bound
                distance_threshold = self.map_x_cell_size / (
                    2 * np.cos(line_perpendicular_angle)
                )
            else:
                # top - bottom bound
                distance_threshold = self.map_y_cell_size / (
                    2 * np.sin(line_perpendicular_angle)
                )

            (
                grid_center_parallel_distance,
                grid_center_perpendicular_distance,
            ) = get_point_line_parallel_perpendicular(
                line_range_centers_flat, line_p, line_v_hat
            )

            line_grid_intersect_mask = (
                np.abs(grid_center_perpendicular_distance) < distance_threshold
            )

            free_grid[
                grid_range[0, 1] : grid_range[1, 1],
                grid_range[0, 0] : grid_range[1, 0],
            ] = np.logical_or(
                free_grid[
                    grid_range[0, 1] : grid_range[1, 1],
                    grid_range[0, 0] : grid_range[1, 0],
                ],
                line_grid_intersect_mask.reshape(line_range_centers.shape[:-1]),
            )

        # Filter out grid which is collided
        free_grid = np.logical_and(free_grid, np.logical_not(collide_grid))

        # Update Map

        # Not free update
        p_not_free = self.map
        p_free = 1.0 - self.map

        # P(not block | ...)
        # P(not block | free) * P(free)
        p_00 = self.lidar_measurement_model[0, 0] * p_free

        # P(not block | not free) * P(not free)
        p_01 = self.lidar_measurement_model[0, 1] * p_not_free

        p_free_given_not_block = np.divide(p_00, p_00 + p_01)
        p_not_free_given_not_block = 1.0 - p_free_given_not_block
        # P(block | ...)
        # P(block | free) * P(free)
        p_10 = self.lidar_measurement_model[1, 0] * p_free

        # P(block | not free) * P(not free)
        p_11 = self.lidar_measurement_model[1, 1] * p_not_free

        p_not_free_given_block = np.divide(p_11, p_10 + p_11)

        # Map represent occupancy probability (not free)
        self.map[collide_grid] = p_not_free_given_block[collide_grid]
        self.map[free_grid] = p_not_free_given_not_block[free_grid]

        # Clip caution_bound
        self.map = np.clip(
            self.map, a_min=self.caution_bound, a_max=1.0 - self.caution_bound
        )

        # Publish Debug Point
        point = Header(
            frame_id=target_frame_id, stamp=self.get_clock().now().to_msg()
        )

        scan_point_local_message = np_to_point_cloud(
            scan_points_homogeneous_transformed[:, :-1],
            self.scan_intensities,
            header=point,
        )

        # self.get_logger().info(f'Pub: ')
        self.point_publisher.publish(scan_point_local_message)

        # Publish Debug Grid
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = target_frame_id
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()

        occupancy_grid.info.origin.position.x = self.map_x_low
        occupancy_grid.info.origin.position.y = self.map_y_low
        occupancy_grid.info.width = len(self.map_x_bounds) - 1
        occupancy_grid.info.height = len(self.map_y_bounds) - 1
        occupancy_grid.info.resolution = self.map_x_cell_size

        # display_grid = 100*collide_grid.astype(np.int8)
        # display_grid = 100*free_grid.astype(np.int8)
        display_grid = (100 * self.map).astype(np.int8)

        occupancy_grid.data = (display_grid).reshape(-1).tolist()

        self.map_publisher.publish(occupancy_grid)


def main(args=None):
    rclpy.init(args=args)

    mapper_node = MapperNode()

    # Spin Node
    rclpy.spin(mapper_node)

    # Destroy node
    mapper_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
