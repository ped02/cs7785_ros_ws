import numpy as np
from typing import Optional, Tuple

from scipy.spatial.transform import Rotation

import rclpy
import rclpy.time

from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Point32, Transform
from sensor_msgs.msg import PointCloud, ChannelFloat32
from nav_msgs.msg import OccupancyGrid, Path


def ros_time_to_seconds(timestamp: rclpy.time.Time):
    seconds, nanoseconds = timestamp.seconds_nanoseconds()

    return seconds + (nanoseconds * 1e-9)


def quat_message_to_rotation(quat_message: Quaternion) -> Rotation:
    return Rotation.from_quat(
        np.array(
            [quat_message.x, quat_message.y, quat_message.z, quat_message.w]
        )
    )


def rotation_to_quat_message(rotation: Rotation) -> Quaternion:
    quat_array = rotation.as_quat()
    return Quaternion(
        x=quat_array[0], y=quat_array[1], z=quat_array[2], w=quat_array[3]
    )


def point_cloud_to_np(point_cloud_message: PointCloud):
    scan_points = np.array(
        [[point.x, point.y] for point in point_cloud_message.points]
    )
    scan_intensities = np.array(
        [channel.values[0] for channel in point_cloud_message.channels]
    )

    return scan_points, scan_intensities


def np_to_point_cloud(
    scan_points,
    scan_intensities,
    header: Optional[Header] = None,
    channel_name: str = 'intensity',
) -> PointCloud:
    message = PointCloud()

    if header is not None:
        message.header = header

    # Points
    points = []
    for point in scan_points:
        # print(point)
        if len(point) >= 3:
            points.append(Point32(x=point[0], y=point[1], z=point[2]))

        else:
            points.append(Point32(x=point[0], y=point[1]))
    message.points = points

    message.channels = [
        ChannelFloat32(values=[v], name=channel_name) for v in scan_intensities
    ]

    return message


def invert_transformation(transform: Transform) -> Transform:
    inverse_transform = Transform()

    quat = transform.rotation
    rotation = quat_message_to_rotation(quat)
    rotation_inv = rotation.inv()

    inverse_transform.rotation = rotation_to_quat_message(rotation_inv)

    translation = np.array(
        [
            [
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ]
        ]
    )

    inversed_translate = -translation @ rotation_inv.as_matrix().T

    inverse_transform.translation.x = inversed_translate[0, 0]
    inverse_transform.translation.y = inversed_translate[0, 1]
    inverse_transform.translation.z = inversed_translate[0, 2]

    return inverse_transform


def parse_occupancy_grid(
    message: OccupancyGrid, shift_center: bool = True
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Parse the grid part of occupancy grid. Assume rotation, only translation offset
    Returns
    -------
    grid: int8 np.ndarray[H, W]
    x_bounds: float np.ndarray[W + 1]
    y_bounds: float np.ndarray[H + 1]
    """
    cell_x_count = message.info.width  # cells
    cell_y_count = message.info.height  # cells

    cell_resolution = message.info.resolution  # m / cell

    grid = np.array(message.data).reshape(cell_y_count, cell_x_count)

    # Assume x and y is aligned with grid
    center_offset = 0.5
    if shift_center:
        center_offset = 0

    x_bounds = (
        np.arange(0, cell_x_count + 1) - center_offset
    ) * cell_resolution + message.info.origin.position.x
    y_bounds = (
        np.arange(0, cell_y_count + 1) - center_offset
    ) * cell_resolution + message.info.origin.position.y

    return grid, x_bounds, y_bounds


def path_to_np_points(message: Path):
    points = [
        [point.pose.position.x, point.pose.position.y]
        for point in message.poses
    ]

    return np.array(points)
