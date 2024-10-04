import numpy as np

import rclpy
from rclpy.node import Node, ParameterDescriptor

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan


class DummyLaserScanPublisher(Node):
    def __init__(self, **kwargs):
        super(DummyLaserScanPublisher, self).__init__(
            'dummy_laser_publisher', **kwargs
        )

        frame_id_parameter_descriptor = ParameterDescriptor(
            description='Target frame_id'
        )
        self.declare_parameter(
            'frame_id', 'offset', frame_id_parameter_descriptor
        )

        self.laser_publisher = self.create_publisher(LaserScan, '/scan', 10)

        PUBLISH_PERIOD_SEC = 0.1
        self.counter = 0
        self.publish_timer = self.create_timer(
            PUBLISH_PERIOD_SEC, self.publish_polygon_array
        )

    def publish_polygon_array(self):
        target_frame_id = (
            self.get_parameter('frame_id').get_parameter_value().string_value
        )
        laser_parameters = [
            {
                'angle_min': 0,
                'angle_max': 2 * np.pi,
                'range_min': 0.0,
                'range_max': 5.0,
                'count': 360,
                'range_function': lambda x: np.ones_like(x),
            }
        ]

        param = laser_parameters[self.counter]

        angle_increment = (param['angle_max'] - param['angle_min']) / param[
            'count'
        ]
        ranges = param['range_function'](
            np.linspace(param['angle_min'], param['angle_max'], param['count'])
        )
        intensities = np.random.uniform(ranges.shape)

        self.laser_publisher.publish(
            LaserScan(
                header=Header(
                    frame_id=target_frame_id,
                    stamp=self.get_clock().now().to_msg(),
                ),
                angle_min=float(param['angle_min']),
                angle_max=float(param['angle_max']),
                angle_increment=float(angle_increment),
                range_min=float(param['range_min']),
                range_max=float(param['range_max']),
                ranges=ranges,
                intensities=intensities,
            )
        )

        self.counter = (self.counter + 1) % len(laser_parameters)


def main(args=None):
    rclpy.init(args=args)

    dummy_lasers_publisher = DummyLaserScanPublisher()

    # Spin Node
    rclpy.spin(dummy_lasers_publisher)

    # Destroy node
    dummy_lasers_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
