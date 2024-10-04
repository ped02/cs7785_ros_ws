import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import PointCloud


def get_clusters(distances, distance_threshold):
    """Distance-based clustering

    Parameters
    ------

    Returns
    -------
    sorted_indices
    cluster_bound_pair: array N x 2 of index bounds
    """
    sorted_indices = np.argsort(distances)
    sorted_distances = distances[sorted_indices]

    sorted_distances_diff = np.diff(sorted_distances)
    cluster_bound_index = np.concatenate(
        [
            [0],
            np.where(sorted_distances_diff > distance_threshold)[0],
            [len(distances)],
        ],
        axis=0,
    )

    cluster_bound_pair = np.stack(
        [cluster_bound_index[:-1], cluster_bound_index[1:]], axis=-1
    )

    return sorted_indices, cluster_bound_pair


class ObjectLocalizeFilter(Node):
    def __init__(self, **kwargs):
        super(ObjectLocalizeFilter, self).__init__(
            'object_localize_filter', **kwargs
        )

        # Subscription
        # TODO: QoS Profile
        self.object_points_subscription = self.create_subscription(
            PointCloud, '/object_points', self.localize_object, 10
        )

        # Publisher
        # TODO: QoS Profile
        self.target_object_publisher = self.create_publisher(
            PointStamped, '/target_object', 10
        )

    def localize_object(self, message: PointCloud):
        target_frame_id = message.header.frame_id

        n_points = len(message.points)

        MIN_POINTS = 10
        if n_points < MIN_POINTS:
            return

        points_2d = np.array(
            [[point.x, point.y, point.z] for point in message.points]
        )
        points_distance_2d = np.sum(
            np.multiply(points_2d, points_2d)[..., :-1], axis=-1
        )

        # Quantile based
        # sorted_indices = np.argsort(points_distance_2d)
        # n_low = min(int(0.3*n_points), n_points)
        # n_high = -min(int(0.3*n_points), n_points)

        # valid_indices = sorted_indices[n_low:n_high]

        NEIGHBOUR_RADIUS = 0.05
        clusters_indices, clusters_bounds_pair = get_clusters(
            points_distance_2d, NEIGHBOUR_RADIUS
        )

        # Get largest cluster
        cluster_size = clusters_bounds_pair[:, 1] - clusters_bounds_pair[:, 0]
        largest_cluster_index = np.argmax(cluster_size)
        largest_cluster_bounds = clusters_bounds_pair[largest_cluster_index]

        valid_indices = clusters_indices[
            largest_cluster_bounds[0] : largest_cluster_bounds[1]
        ]

        valid_mask = np.zeros_like(points_distance_2d, dtype=bool)
        valid_mask[valid_indices] = True

        valid_points = points_2d[valid_mask]
        # Assume points is clustered
        average_point = np.mean(valid_points, axis=0)

        # self.get_logger().info(f'{average_point.shape}')

        point_stamped = PointStamped(
            header=Header(
                frame_id=target_frame_id, stamp=self.get_clock().now().to_msg()
            ),
            point=Point(
                x=average_point[0], y=average_point[1], z=average_point[2]
            ),
        )

        self.target_object_publisher.publish(point_stamped)


def main(args=None):
    rclpy.init(args=args)

    object_localize_filter = ObjectLocalizeFilter()

    # Spin Node
    rclpy.spin(object_localize_filter)

    # Destroy node
    object_localize_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
