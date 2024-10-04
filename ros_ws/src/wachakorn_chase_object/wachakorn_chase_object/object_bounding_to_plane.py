from collections.abc import Mapping, Collection
from typing import Callable

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node, ParameterDescriptor, Parameter, SetParametersResult

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import Header
from geometry_msgs.msg import PolygonStamped, Vector3, Point32, Polygon

from robot_interfaces.msg import (
    PlaneNormalDistanceArrayStamped,
    PolygonArrayStamped,
)

from .parameter_utils import validate_positive_double, validate_positive_int

from .camera_utils import (
    get_camera_intrinsic,
    bounding_polygon_to_inward_planes,
    plane_to_polygon,
    pixels_to_world_point,
    to_homogeneous,
)


def polygon_list_to_message(polygons, frame_id, timestamp):
    return PolygonArrayStamped(
        header=Header(frame_id=frame_id, stamp=timestamp),
        polygons=[
            Polygon(
                points=[
                    Point32(
                        x=float(point[0]), y=float(point[1]), z=float(point[2])
                    )
                    for point in polygon
                ]
            )
            for polygon in polygons
        ],
    )


class BoundingToPlane(Node):
    def __init__(self, **kwargs):
        super(BoundingToPlane, self).__init__('bounding_to_plane', **kwargs)

        # Parameters
        parameter_validator: Mapping[
            str, Callable[[Parameter], SetParametersResult]
        ] = {}

        # Frame to represent planes in
        target_frame_id_parameter_descriptor = ParameterDescriptor(
            description='Frame to represent frame in'
        )
        self.declare_parameter(
            'target_frame_id', 'base_scan', target_frame_id_parameter_descriptor
        )

        # Camera and Image parameters
        camera_focal_length_parameter_descriptor = ParameterDescriptor(
            description='Camera Focal Length in meters. Must be positive number'
        )
        self.declare_parameter(
            'focal_length', 0.00304, camera_focal_length_parameter_descriptor
        )

        camera_sensor_width_parameter_descriptor = ParameterDescriptor(
            description='Camera Sensor Width in meters. Must be positive number'
        )
        self.declare_parameter(
            'sensor_width', 0.00368, camera_sensor_width_parameter_descriptor
        )

        camera_sensor_height_parameter_descriptor = ParameterDescriptor(
            description='Camera Sensor Height in meters. Must be positive number'
        )
        self.declare_parameter(
            'sensor_height', 0.00276, camera_sensor_height_parameter_descriptor
        )

        parameter_validator['focal_length'] = validate_positive_double
        parameter_validator['sensor_width'] = validate_positive_double
        parameter_validator['sensor_height'] = validate_positive_double

        image_width_parameter_descriptor = ParameterDescriptor(
            description='Image width in pixels. Must be positive number'
        )
        self.declare_parameter(
            'image_width', 320, image_width_parameter_descriptor
        )

        image_height_parameter_descriptor = ParameterDescriptor(
            description='Image height in pixels. Must be positive number'
        )
        self.declare_parameter(
            'image_height', 240, image_height_parameter_descriptor
        )

        parameter_validator['image_width'] = validate_positive_int
        parameter_validator['image_height'] = validate_positive_int

        # Visualization
        display_plane_parameter_descriptor = ParameterDescriptor(
            description='Whether to publish plane result. Bool'
        )
        self.declare_parameter(
            'display_plane', False, display_plane_parameter_descriptor
        )

        display_x_lower_bounds_parameter_descriptor = ParameterDescriptor(
            description='Display x lower bounds in meters.'
        )
        self.declare_parameter(
            'display_x_lower_bound',
            -1.0,
            display_x_lower_bounds_parameter_descriptor,
        )

        display_x_upper_bounds_parameter_descriptor = ParameterDescriptor(
            description='Display x upper bounds in meters.'
        )
        self.declare_parameter(
            'display_x_upper_bound',
            1.0,
            display_x_upper_bounds_parameter_descriptor,
        )

        display_y_lower_bounds_parameter_descriptor = ParameterDescriptor(
            description='Display y lower bounds in meters.'
        )
        self.declare_parameter(
            'display_y_lower_bound',
            -1.0,
            display_y_lower_bounds_parameter_descriptor,
        )

        display_y_upper_bounds_parameter_descriptor = ParameterDescriptor(
            description='Display y upper bounds in meters.'
        )
        self.declare_parameter(
            'display_y_upper_bound',
            1.0,
            display_y_upper_bounds_parameter_descriptor,
        )

        display_z_lower_bounds_parameter_descriptor = ParameterDescriptor(
            description='Display z lower bounds in meters.'
        )
        self.declare_parameter(
            'display_z_lower_bound',
            -1.0,
            display_z_lower_bounds_parameter_descriptor,
        )

        display_z_upper_bounds_parameter_descriptor = ParameterDescriptor(
            description='Display z upper bounds in meters.'
        )
        self.declare_parameter(
            'display_z_upper_bound',
            1.0,
            display_z_upper_bounds_parameter_descriptor,
        )

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
        self.calculate_camera_parameters()

        self._prev_camera_focal_length = self.camera_focal_length
        self._prev_sensor_width = self.sensor_width
        self._prev_sensor_height = self.sensor_height
        self._prev_image_width = self.image_width
        self._prev_image_height = self.image_height

        self._prev_target_frame_id = self.target_frame_id
        self._prev_display_x_lower_bounds = self.display_x_lower_bounds
        self._prev_display_x_upper_bounds = self.display_x_upper_bounds
        self._prev_display_y_lower_bounds = self.display_y_lower_bounds
        self._prev_display_y_upper_bounds = self.display_y_upper_bounds
        self._prev_display_z_lower_bounds = self.display_z_lower_bounds
        self._prev_display_z_upper_bounds = self.display_z_upper_bounds

        # Transform
        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer, self)

        # Subscriber
        # TODO: Change QOS
        self.bouding_polygon_subscriber = self.create_subscription(
            PolygonStamped,
            '/segmenter/image_bounding',
            self.process_polygon,
            10,
        )

        # Publisher
        # TODO: Change QOS
        self.bounding_polygon_plane_publisher = self.create_publisher(
            PlaneNormalDistanceArrayStamped, '/image_bounding_planes', 10
        )

        # TODO: Change QOS
        self.display_plane_polygon_publisher = self.create_publisher(
            PolygonArrayStamped, '/image_bounding_planes_polygon', 10
        )

        self.display_object_reproject = self.create_publisher(
            PolygonStamped, '/image_bounding_reproject', 10
        )

    def update_parameters(self) -> None:
        self.target_frame_id = (
            self.get_parameter('target_frame_id')
            .get_parameter_value()
            .string_value
        )

        self.camera_focal_length = (
            self.get_parameter('focal_length')
            .get_parameter_value()
            .double_value
        )
        self.sensor_width = (
            self.get_parameter('sensor_width')
            .get_parameter_value()
            .double_value
        )
        self.sensor_height = (
            self.get_parameter('sensor_height')
            .get_parameter_value()
            .double_value
        )

        self.image_width = (
            self.get_parameter('image_width')
            .get_parameter_value()
            .integer_value
        )
        self.image_height = (
            self.get_parameter('image_height')
            .get_parameter_value()
            .integer_value
        )

        self.display_plane = (
            self.get_parameter('display_plane').get_parameter_value().bool_value
        )
        self.display_x_lower_bounds = (
            self.get_parameter('display_x_lower_bound')
            .get_parameter_value()
            .double_value
        )
        self.display_x_upper_bounds = (
            self.get_parameter('display_x_upper_bound')
            .get_parameter_value()
            .double_value
        )
        self.display_y_lower_bounds = (
            self.get_parameter('display_y_lower_bound')
            .get_parameter_value()
            .double_value
        )
        self.display_y_upper_bounds = (
            self.get_parameter('display_y_upper_bound')
            .get_parameter_value()
            .double_value
        )
        self.display_z_lower_bounds = (
            self.get_parameter('display_z_lower_bound')
            .get_parameter_value()
            .double_value
        )
        self.display_z_upper_bounds = (
            self.get_parameter('display_z_upper_bound')
            .get_parameter_value()
            .double_value
        )

    def parameter_changed(self) -> bool:
        """Determine if parameter changed values. Mutate internal states"""

        parameter_not_changed = (
            (self._prev_camera_focal_length == self.camera_focal_length)
            and (self._prev_sensor_width == self.sensor_width)
            and (self._prev_sensor_height == self.sensor_height)
            and (self._prev_image_width == self.image_width)
            and (self._prev_image_height == self.image_height)
            and (self._prev_target_frame_id == self.target_frame_id)
            and (
                self._prev_display_x_lower_bounds == self.display_x_lower_bounds
            )
            and (
                self._prev_display_x_upper_bounds == self.display_x_upper_bounds
            )
            and (
                self._prev_display_y_lower_bounds == self.display_y_lower_bounds
            )
            and (
                self._prev_display_y_upper_bounds == self.display_y_upper_bounds
            )
            and (
                self._prev_display_z_lower_bounds == self.display_z_lower_bounds
            )
            and (
                self._prev_display_z_upper_bounds == self.display_z_upper_bounds
            )
        )

        if not parameter_not_changed:
            self._prev_camera_focal_length = self.camera_focal_length
            self._prev_sensor_width = self.sensor_width
            self._prev_sensor_height = self.sensor_height
            self._prev_image_width = self.image_width
            self._prev_image_height = self.image_height

            self._prev_target_frame_id = self.target_frame_id
            self._prev_display_x_lower_bounds = self.display_x_lower_bounds
            self._prev_display_x_upper_bounds = self.display_x_upper_bounds
            self._prev_display_y_lower_bounds = self.display_y_lower_bounds
            self._prev_display_y_upper_bounds = self.display_y_upper_bounds
            self._prev_display_z_lower_bounds = self.display_z_lower_bounds
            self._prev_display_z_upper_bounds = self.display_z_upper_bounds

        return not parameter_not_changed

    def calculate_camera_parameters(self):
        self.camera_intrinsics = get_camera_intrinsic(
            camera_f=self.camera_focal_length,
            sensor_width=self.sensor_width,
            sensor_height=self.sensor_height,
            image_width=self.image_width,
            image_height=self.image_height,
        )

        self.camera_intrinsics_inverse = np.linalg.inv(self.camera_intrinsics)

        self.display_x_bounds = np.array(
            [self.display_x_lower_bounds, self.display_x_upper_bounds]
        )
        self.display_y_bounds = np.array(
            [self.display_y_lower_bounds, self.display_y_upper_bounds]
        )
        self.display_z_bounds = np.array(
            [self.display_z_lower_bounds, self.display_z_upper_bounds]
        )

        # Maybe calculate the new image bound to use?

    def process_polygon(self, polygon_message: PolygonStamped):
        # Check parameters
        self.update_parameters()

        if self.parameter_changed():
            # Re-calculate matrices
            self.calculate_camera_parameters()

        # Camera transform may change

        bounding_source_frame_id = polygon_message.header.frame_id

        camera_transform = None
        try:
            camera_transform = self.transform_buffer.lookup_transform(
                self.target_frame_id,
                bounding_source_frame_id,
                self.get_clock().now().to_msg(),
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform '{bounding_source_frame_id}' to '{self.target_frame_id}': {ex}"
            )

        if camera_transform is None:
            # Fail: wait to try again
            return

        camera_frame_quat = Rotation.from_quat(
            np.array(
                [
                    camera_transform.transform.rotation.x,
                    camera_transform.transform.rotation.y,
                    camera_transform.transform.rotation.z,
                    camera_transform.transform.rotation.w,
                ]
            )
        )

        image_plane_realign_matrix = np.eye(3, 3, dtype=np.float32)
        image_plane_realign_matrix[:3, 1] *= -1.0

        camera_realign_matrix = Rotation.from_euler(
            'xz', [-np.pi / 2, np.pi / 2]
        ).as_matrix()

        camera_frame = np.eye(4, 4, dtype=np.float32)
        camera_frame[:3, :3] = (
            camera_frame_quat.as_matrix()
            @ camera_realign_matrix
            @ image_plane_realign_matrix
        )
        camera_frame[:3, 3] = np.array(
            [
                camera_transform.transform.translation.x,
                camera_transform.transform.translation.y,
                camera_transform.transform.translation.z,
            ]
        )

        # print(camera_frame)

        # camera_frame[:3, :3] = camera_realign.as_matrix()

        # Extract bounding points
        polygon_points_msg = polygon_message.polygon.points
        if len(polygon_points_msg) == 0:
            # No polygon received
            return

        polygon_points = np.array(
            [[point.x, point.y] for point in polygon_points_msg],
            dtype=np.float32,
        )

        planes_n, planes_d = bounding_polygon_to_inward_planes(
            polygon_points,
            self.camera_intrinsics_inverse,
            -self.camera_focal_length,
            camera_frame,
        )

        planes_n_vector = [
            Vector3(x=x, y=y, z=z) for (x, y, z) in planes_n.tolist()
        ]

        planes_message_stamped = PlaneNormalDistanceArrayStamped(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id=self.target_frame_id,
            ),
            unit_normal_vectors=planes_n_vector,
            distances=planes_d.tolist(),
        )

        self.bounding_polygon_plane_publisher.publish(planes_message_stamped)

        # Display
        if self.display_plane:
            plane_polygon = plane_to_polygon(
                planes_n=planes_n,
                planes_d=planes_d,
                x_bounds=self.display_x_bounds,
                y_bounds=self.display_y_bounds,
                z_bounds=self.display_z_bounds,
            ).tolist()

            self.display_plane_polygon_publisher.publish(
                polygon_list_to_message(
                    plane_polygon,
                    self.target_frame_id,
                    self.get_clock().now().to_msg(),
                )
            )

            world_points_homogeneous = pixels_to_world_point(
                camera_frame,
                self.camera_intrinsics_inverse,
                to_homogeneous(polygon_points),
                -0.5,
            )
            self.display_object_reproject.publish(
                PolygonStamped(
                    header=Header(
                        frame_id=self.target_frame_id,
                        stamp=self.get_clock().now().to_msg(),
                    ),
                    polygon=Polygon(
                        points=[
                            Point32(x=x, y=y, z=z)
                            for x, y, z, w in world_points_homogeneous
                        ]
                    ),
                )
            )


def main(args=None):
    rclpy.init(args=args)

    bounding_to_plane = BoundingToPlane()

    # Spin Node
    rclpy.spin(bounding_to_plane)

    # Destroy node
    bounding_to_plane.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
