import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud


class ScanPointCloudWindowNode(Node):
    ## ASSUME SAME POINT FRAME
    def __init__(self, **kwargs):
        super(ScanPointCloudWindowNode, self).__init__(
            'scan_point_cloud_window_node', **kwargs
        )

        # TODO: Parameters
        self.window_size = 3
        self.window_stale_time_sec = 100000000000000000
        # self.window_stale_time_sec = 0.1

        # Subscriber
        point_cloud_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.point_cloud_subscriptions = self.create_subscription(
            PointCloud,
            '/scan/point_cloud',
            self.roll_window,
            point_cloud_qos_profile,
        )

        # Publishser
        self.window_point_cloud_publisher = self.create_publisher(
            PointCloud, '/scan/point_cloud/window', point_cloud_qos_profile
        )

        self.reset()

    def reset(self):
        self.point_data = []
        self.channel_data = []
        self.point_time = []

    def pop_data(self):
        self.point_data = self.point_data[1:]
        self.channel_data = self.channel_data[1:]
        self.point_time = self.point_time[1:]

    def roll_window(self, message: PointCloud):
        frame_id = message.header.frame_id

        # frame_id = 'map'

        message_time = rclpy.time.Time.from_msg(message.header.stamp)

        if len(self.point_data) == self.window_size:
            self.pop_data()

        while len(self.point_time) > 0:
            time_difference_seconds = (
                self.get_clock().now() - self.point_time[0]
            ).nanoseconds * 1e-9

            if time_difference_seconds > self.window_stale_time_sec:
                self.pop_data()
            else:
                break

        self.point_data.append(message.points)
        self.channel_data.append(message.channels)
        self.point_time.append(message_time)

        point_cloud_message = PointCloud(
            header=Header(frame_id=frame_id, stamp=message.header.stamp),
            points=[point for points in self.point_data for point in points],
            channels=[
                channel
                for channels in self.channel_data
                for channel in channels
            ],
        )

        self.window_point_cloud_publisher.publish(point_cloud_message)


def main(args=None):
    rclpy.init(args=args)
    scan_point_cloud_window = ScanPointCloudWindowNode()
    # Spin Node
    rclpy.spin(scan_point_cloud_window)

    # Destroy node
    scan_point_cloud_window.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
