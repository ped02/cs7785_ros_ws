import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32


class ScanToPointCloud(Node):
    def __init__(self, **kwargs):
        super(ScanToPointCloud, self).__init__('scan_to_point_cloud', **kwargs)

        # Parameters
        # self.too_close_distance_radius_m = 0.25

        # Transforms
        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer, self)

        # Laser Subscripber
        laser_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.laser_scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.process_laser_scan, laser_qos_profile
        )

        # Point Cloud Publisher
        point_cloud_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.point_cloud_publisher = self.create_publisher(
            PointCloud, '/scan/point_cloud', point_cloud_qos_profile
        )

        # self.too_close_to_obstacle_publisher = self.create_publisher(
        #     Bool, '/obstacle_too_close', 3
        # )

    def process_laser_scan(self, message: LaserScan):
        laser_distances = np.array(message.ranges, dtype=np.float32)
        laser_angles = np.linspace(
            message.angle_min,
            message.angle_max,
            len(laser_distances),
            dtype=np.float32,
        )
        laser_intensities = np.array(message.intensities, dtype=np.float32)

        # Filter nans
        nan_mask = np.isnan(laser_distances)

        laser_distances = laser_distances[np.logical_not(nan_mask)]
        laser_angles = laser_angles[np.logical_not(nan_mask)]
        laser_intensities = laser_intensities[np.logical_not(nan_mask)]

        # Filter out min max
        in_range_mask = np.logical_and(
            message.angle_min <= laser_distances,
            laser_distances <= message.angle_max,
        )

        laser_distances = laser_distances[in_range_mask]
        laser_angles = laser_angles[in_range_mask]
        laser_intensities = laser_intensities[in_range_mask]

        # self.get_logger().info(f'{np.min(laser_distances)=} {bool(np.any(laser_distances <= self.too_close_distance_radius_m))=}')
        # self.too_close_to_obstacle_publisher.publish(Bool(data=bool(np.any(laser_distances <= self.too_close_distance_radius_m))))

        # Convert to point cloud
        points = np.stack(
            [
                laser_distances * np.cos(laser_angles),
                laser_distances * np.sin(laser_angles),
                np.zeros_like(laser_angles),
            ],
            axis=-1,
        )

        point_cloud_message = PointCloud(
            header=message.header,
            points=[
                Point32(x=float(x), y=float(y), z=float(z))
                for x, y, z in points
            ],
            channels=[
                ChannelFloat32(name='intensity', values=[float(intensity)])
                for intensity in laser_intensities
            ],
        )

        # point_cloud_message.header.stamp = self.get_clock().now().to_msg()

        self.point_cloud_publisher.publish(point_cloud_message)


def main(args=None):
    rclpy.init(args=args)
    scan_to_point_cloud_node = ScanToPointCloud()

    # Spin Node
    rclpy.spin(scan_to_point_cloud_node)

    # Destroy node
    scan_to_point_cloud_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
