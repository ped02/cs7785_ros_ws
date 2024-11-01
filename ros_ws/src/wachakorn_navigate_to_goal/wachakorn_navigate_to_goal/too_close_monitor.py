import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud

from wachakorn_chase_object.ros_utils import point_cloud_to_np


class TooCloseMonitor(Node):
    def __init__(self, **kwargs):
        super(TooCloseMonitor, self).__init__('too_close_monitor', **kwargs)

        self.too_close_areas = [
            ((-0.15, 0.15), (-0.17, 0.17)),  # (x_low, x_high). (y_low, y_high)
            ((0.15, 0.25), (-0.17, 0.17)),  # (x_low, x_high). (y_low, y_high)
            ((-0.25, -0.15), (-0.17, 0.17)),  # (x_low, x_high). (y_low, y_high)
        ]

        point_cloud_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.scan_point_subscriber = self.create_subscription(
            PointCloud,
            '/scan/point_cloud',
            self.process_point_cloud,
            point_cloud_qos_profile,
        )

        self.too_close_to_obstacle_publisher = self.create_publisher(
            Bool, '/obstacle_too_close', 3
        )

    def process_point_cloud(self, message: PointCloud):
        point_cloud, point_intensities = point_cloud_to_np(message)

        too_close = False

        for (area_x_low, area_x_high), (
            area_y_low,
            area_y_high,
        ) in self.too_close_areas:
            x_in_mask = np.logical_and(
                area_x_low < point_cloud[:, 0], point_cloud[:, 0] <= area_x_high
            )
            y_in_mask = np.logical_and(
                area_y_low < point_cloud[:, 1], point_cloud[:, 1] <= area_y_high
            )

            in_mask = np.logical_and(x_in_mask, y_in_mask)

            too_close = too_close | np.any(in_mask)

        self.too_close_to_obstacle_publisher.publish(Bool(data=bool(too_close)))


def main(args=None):
    rclpy.init(args=args)

    too_close_monitor = TooCloseMonitor()

    # Spin Node
    rclpy.spin(too_close_monitor)

    # Destroy node
    too_close_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
