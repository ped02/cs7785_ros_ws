import numpy as np

from scipy.spatial.transform import Rotation

import rclpy
import rclpy.time

from geometry_msgs.msg import Quaternion


def ros_time_to_seconds(timestamp: rclpy.time.Time):
    seconds, nanoseconds = timestamp.seconds_nanoseconds()

    return seconds + (nanoseconds * 1e-9)


def quat_message_to_rotation(quat_message: Quaternion) -> Rotation:
    return Rotation.from_quat(
        np.array(
            [quat_message.x, quat_message.y, quat_message.z, quat_message.w]
        )
    )
