import math


import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Polygon, Point32
from sensor_msgs.msg import PointCloud
from robot_interfaces.msg import PolygonArrayStamped

from wachakorn_chase_object.ros_utils import point_cloud_to_np

from .geometry_utils import (
    points_to_distance_angle,
    cluster_scan,
    get_cluster_lines,
    get_corner_points_lines_consecutive,
)


class ScanSegmenterNode(Node):
    def __init__(self, **kwargs):
        super(ScanSegmenterNode, self).__init__('scan_segmenter_node', **kwargs)

        # TODO: Parameters

        # Subscriber
        point_cloud_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.point_cloud_subscriber = self.create_subscription(
            PointCloud,
            '/scan/point_cloud',
            self.segment_callback,
            point_cloud_qos_profile,
        )

        # Lines Publisher
        lines_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.lines_publisher = self.create_publisher(
            PolygonArrayStamped, '/scan_segmenter/lines', lines_qos_profile
        )

        # Intersection
        self.intersection_point_publisher = self.create_publisher(
            PointCloud, '/scan_segmenter/intersection_point', lines_qos_profile
        )

    def segment_callback(self, message: PointCloud):
        scan_points, scan_intensities = point_cloud_to_np(message)
        scan_r, scan_theta = points_to_distance_angle(scan_points)

        # scan_sorted_indices, cluster_bound_pairs = cluster_scan(scan_r, scan_theta, scan_points, max_neighbour_distance=0.3, min_points=3)
        scan_sorted_indices, cluster_bound_pairs = cluster_scan(
            scan_r,
            scan_theta,
            scan_points,
            max_neighbour_distance=0.15,
            min_points=3,
        )

        # scan_r_sorted = scan_r[scan_sorted_indices]
        # scan_theta_sorted = scan_theta[scan_sorted_indices]
        scan_points_sorted = scan_points[scan_sorted_indices]

        # print(f'Cluster Found: {len(cluster_bound_pairs)}')

        clusters = []
        for cluster_bound in cluster_bound_pairs:
            lower_bound, upper_bound = cluster_bound
            clusters.append(
                scan_points_sorted.take(
                    range(lower_bound, upper_bound), axis=0, mode='wrap'
                )
            )

        # Get Line for each clusters
        # cluster_lines = [split_and_merge_points(cluster, 0.02, 0.01, 3) for cluster in clusters]

        # inlier_distance_threshold = 0.15
        # merge_distance_threshold = 0.2
        # ransac_inlier_distance_threshold = 0.1
        # fit_line_max_variance_thresold = 3.0
        # min_line_points = 10
        # ransac_sample_size = 5

        # ransac_inlier_probability: Number = 0.5
        # ransac_success_probability: Number = 0.99
        # ransac_max_iterations: int = 10000

        # cluster_lines = [
        #     split_and_merge_points_ransac(cluster,
        #                                 inlier_distance_threshold = inlier_distance_threshold,
        #                                 merge_distance_threshold = merge_distance_threshold,
        #                                 ransac_inlier_distance_threshold = ransac_inlier_distance_threshold,
        #                                 fit_line_max_variance_thresold = fit_line_max_variance_thresold,
        #                                 min_line_points = min_line_points,
        #                                 ransac_sample_size = ransac_sample_size,
        #                                 ransac_inlier_probability = ransac_inlier_probability,
        #                                 ransac_success_probability = ransac_success_probability,
        #                                 ransac_max_iterations = ransac_max_iterations
        #                                 ) for cluster in clusters
        # ]

        cluster_lines = [
            get_cluster_lines(
                cluster,
                30 * math.pi / 180,
                2,
                0.3,
                merge_endpoint_distance_threshold=0.05,
            )
            for cluster in clusters
        ]

        # Get intersection points for each clusters
        # intersection_points = [get_corner_points_lines(line, 0.05) for line in cluster_lines]
        intersection_points = [
            get_corner_points_lines_consecutive(line, 0.05)
            for line in cluster_lines
        ]

        # Form intersection line

        display_polygons = []
        for lines in cluster_lines:
            for line_bounds, _ in lines:
                display_polygons.append(
                    Polygon(
                        points=[
                            Point32(x=float(x), y=float(y), z=0.0)
                            for x, y in line_bounds
                        ]
                    )
                )

        display_line_message = PolygonArrayStamped(
            header=message.header, polygons=display_polygons
        )

        # display_line_message.header.frame_id='fixed'

        self.lines_publisher.publish(display_line_message)

        display_corners = []
        for corner in intersection_points:
            if len(corner) > 0:
                for points, _ in corner:
                    display_corners.extend(
                        [
                            Point32(x=float(x), y=float(y), z=0.0)
                            for x, y in points
                        ]
                    )

        display_corner_message = PointCloud(
            header=message.header, points=display_corners
        )

        # display_corner_message.header.frame_id='fixed'

        self.intersection_point_publisher.publish(display_corner_message)


def main(args=None):
    rclpy.init(args=args)

    scan_segmenter_node = ScanSegmenterNode()

    # Spin Node
    rclpy.spin(scan_segmenter_node)

    # Destroy node
    scan_segmenter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
