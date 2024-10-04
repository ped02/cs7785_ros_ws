from collections.abc import Collection, Mapping
from typing import Callable

import numpy as np

import rclpy
import rclpy.time
from rclpy.node import (
    Node,
    Parameter,
    SetParametersResult,
    ParameterDescriptor,
    MutuallyExclusiveCallbackGroup,
)

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import Header
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud

from robot_interfaces.msg import PlaneNormalDistanceArrayStamped

from .ros_utils import quat_message_to_rotation
from .parameter_utils import validate_positive_double
from .camera_utils import get_plane_side_mask, to_homogeneous


class FilterScanPoints(Node):
    def __init__(self, **kwargs):
        super(FilterScanPoints, self).__init__('filter_scan_point', **kwargs)

        parameter_validator: Mapping[
            str, Callable[[Parameter], SetParametersResult]
        ] = {}

        max_sync_time_difference_sec_parameter_descriptor = ParameterDescriptor(
            description='Max time difference for syncing locations in seconds.'
        )
        self.declare_parameter(
            'max_sync_time_difference_sec',
            0.2,
            max_sync_time_difference_sec_parameter_descriptor,
        )

        parameter_validator[
            'max_sync_time_difference_sec'
        ] = validate_positive_double

        def validate_parameters(
            parameters: Collection[Parameter],
        ) -> SetParametersResult:
            for parameter in parameters:
                validator = parameter_validator.get(parameter.name, None)

                if validator:
                    validate_result = validator(parameter)
                    if not validate_result.successful:
                        return validate_result

            return SetParametersResult(successful=True)

        self.add_on_set_parameters_callback(validate_parameters)

        self.update_parameters()

        # Transforms
        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer, self)

        # Subscriptions

        sensor_callback_group = MutuallyExclusiveCallbackGroup()

        laser_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.laser_scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.set_laser_scan,
            laser_qos_profile,
            callback_group=sensor_callback_group,
        )
        self.reset_laser_scan()

        planes_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.plane_subscription = self.create_subscription(
            PlaneNormalDistanceArrayStamped,
            '/image_bounding_planes',
            self.set_planes,
            planes_qos_profile,
            callback_group=sensor_callback_group,
        )
        self.reset_planes()

        # Process Timer
        PROCESS_TIMER_POLL_PERIOD_SEC = 0.02
        self.process_timer = self.create_timer(
            PROCESS_TIMER_POLL_PERIOD_SEC,
            self.process_points,
            callback_group=sensor_callback_group,
        )

        self.processed = False

        # Publisher
        self.point_cloud = self.create_publisher(
            PointCloud, '/object_points', 10
        )
        self.filtered_scan_publisher = self.create_publisher(
            PointCloud, '/scan_filtered', 10
        )

    def update_parameters(self):
        self.max_sync_time_difference_sec = (
            self.get_parameter('max_sync_time_difference_sec')
            .get_parameter_value()
            .double_value
        )

    def reset_laser_scan(self):
        self.laser_frame_id = ''
        self.laser_timestamp = rclpy.time.Time(
            clock_type=self.get_clock().clock_type
        )

        self.laser_angles = np.zeros(0)
        self.laser_distances = np.zeros(0)

    def set_laser_scan(self, message: LaserScan):
        self.laser_frame_id = message.header.frame_id
        self.laser_timestamp = rclpy.time.Time.from_msg(message.header.stamp)

        self.laser_distances = np.array(message.ranges, dtype=np.float32)
        self.laser_angles = np.linspace(
            message.angle_min,
            message.angle_max,
            len(self.laser_distances),
            dtype=np.float32,
        )

        self.processed = False

        # Filter nans
        nan_mask = np.isnan(self.laser_distances)

        self.laser_distances = self.laser_distances[np.logical_not(nan_mask)]
        self.laser_angles = self.laser_angles[np.logical_not(nan_mask)]

        # Filter out min max
        in_range_mask = np.logical_and(
            message.angle_min <= self.laser_distances,
            self.laser_distances <= message.angle_max,
        )

        self.laser_distances = self.laser_distances[in_range_mask]
        self.laser_angles = self.laser_angles[in_range_mask]

    def reset_planes(self):
        self.planes_frame_id = ''
        self.planes_timestamp = rclpy.time.Time(
            clock_type=self.get_clock().clock_type
        )

        self.planes_n = np.zeros((0, 0), dtype=np.float32)
        self.planes_d = np.zeros((0), dtype=np.float32)

    def set_planes(self, message: PlaneNormalDistanceArrayStamped):
        self.planes_frame_id = message.header.frame_id
        self.planes_timestamp = rclpy.time.Time.from_msg(message.header.stamp)

        self.planes_n = np.array(
            [
                [vector.x, vector.y, vector.z]
                for vector in message.unit_normal_vectors
            ],
            dtype=np.float32,
        )
        self.planes_d = np.array(message.distances, dtype=np.float32)

        self.processed = False

    def process_points(self):
        self.update_parameters()

        # Check timestamp
        time_difference_seconds = (
            self.laser_timestamp - self.planes_timestamp
        ).nanoseconds * 1e-9

        if abs(time_difference_seconds) > self.max_sync_time_difference_sec:
            # Data too stale
            # self.get_logger().info(f'Stale  [{abs(time_difference_seconds)}]')
            return

        if self.processed:
            return

        # self.get_logger().info('Process')
        # Transform points into plane frame

        target_frame_id = self.planes_frame_id
        source_frame_id = self.laser_frame_id

        point_transform = None
        try:
            point_transform = self.transform_buffer.lookup_transform(
                target_frame_id,
                source_frame_id,
                self.get_clock().now().to_msg(),
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform '{source_frame_id}' to '{target_frame_id}': {ex}"
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

        laser_points = np.stack(
            [
                np.multiply(self.laser_distances, np.cos(self.laser_angles)),
                np.multiply(self.laser_distances, np.sin(self.laser_angles)),
                np.zeros_like(self.laser_distances),
            ],
            axis=-1,
        )

        laser_points_homogeneous = to_homogeneous(laser_points)
        laser_points_homogeneous_transformed = (
            laser_points_homogeneous @ point_transform_matrix.T
        )

        object_point_mask = get_plane_side_mask(
            laser_points_homogeneous_transformed[..., :-1],
            self.planes_n,
            self.planes_d,
            same_side_as_normal=False,
        )

        object_points = laser_points[object_point_mask]

        point_cloud_message = PointCloud(
            header=Header(
                frame_id=target_frame_id, stamp=self.get_clock().now().to_msg()
            ),
            points=[
                Point32(x=float(x), y=float(y), z=float(z))
                for (x, y, z) in object_points
            ],
        )

        self.point_cloud.publish(point_cloud_message)

        self.filtered_scan_publisher.publish(
            PointCloud(
                header=Header(
                    frame_id=target_frame_id,
                    stamp=self.get_clock().now().to_msg(),
                ),
                points=[
                    Point32(x=float(x), y=float(y), z=float(z))
                    for (x, y, z) in laser_points
                ],
            )
        )
        self.processed = True


def main(args=None):
    rclpy.init(args=args)

    filter_scan_points = FilterScanPoints()

    # Spin Node
    rclpy.spin(filter_scan_points)

    # Destroy node
    filter_scan_points.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
