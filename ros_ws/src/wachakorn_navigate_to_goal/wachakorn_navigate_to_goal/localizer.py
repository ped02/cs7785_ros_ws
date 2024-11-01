import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster, TransformStamped

from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Vector3
from nav_msgs.msg import Odometry
from robot_interfaces.srv import ResetState

from wachakorn_chase_object.ros_utils import (
    quat_message_to_rotation,
    rotation_to_quat_message,
    invert_transformation,
)


def rotation_2d(angle_radians):
    cos = np.cos(angle_radians)
    sin = np.sin(angle_radians)
    return np.array([[cos, -sin], [sin, cos]])


def rotation_2d_homogeneous(angle_radians):
    cos = np.cos(angle_radians)
    sin = np.sin(angle_radians)
    z = np.zeros_like(angle_radians)
    o = np.ones_like(angle_radians)
    return np.array([[cos, -sin, z], [sin, cos, z], [z, z, o]])


def rotate_translate_homogeneous(angle_radians, translate):
    transform = rotation_2d_homogeneous(angle_radians)
    transform[:2, 2] = translate
    return transform


class LocalizerNode(Node):
    def __init__(self, **kwargs):
        node_name = 'localizer_node'
        super(LocalizerNode, self).__init__(node_name, **kwargs)

        # self.parent_frame_id = 'world'
        self.base_frame_id = 'localize_base'
        self.odom_frame_id = 'localize_odom'

        self.base_link_frame_id = 'base_link'

        # Pose publisher
        # TODO: odom qos
        self.odom_publisher = self.create_publisher(
            Odometry, '/estimate_odom', 10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # Pose Subscriber
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.update_odometry, 10
        )

        self.reset_state_service = self.create_service(
            ResetState,
            f'/{node_name}/reset_state',
            self.reset_state_service_callback,
        )

        self.current_position = None
        self.current_rotation = None

        self.reset_state()

    def reset_state(self):
        if self.current_position is None:
            self.base_pose_position = np.zeros((1, 2), np.float64)
            self.base_pose_rotation = np.eye(2, 2, dtype=np.float64)

        else:
            self.base_pose_position = self.current_position
            self.base_pose_rotation = self.current_rotation
            self.get_logger().info(
                f'Setting to current: {self.current_position=}, {self.current_rotation=}'
            )

        self.base_pose_transform = np.eye(3, 3, dtype=np.float64)
        self.base_pose_transform[:2, :2] = self.base_pose_rotation[:2, :2]
        rotation_quat = rotation_to_quat_message(
            Rotation.from_matrix(self.base_pose_transform)
        )
        self.base_pose_transform[:2, 2] = np.squeeze(self.base_pose_position)

        self.base_transform_inverse = np.linalg.inv(self.base_pose_transform)

        # Ros2 TF
        current_t_to_base_link = TransformStamped(
            header=Header(
                frame_id=self.base_link_frame_id,
                stamp=self.get_clock().now().to_msg(),
            ),
            child_frame_id=self.odom_frame_id,
        )
        self.tf_broadcaster.sendTransform(current_t_to_base_link)

        self.base_t = TransformStamped(
            header=Header(
                frame_id=self.odom_frame_id,
                stamp=self.get_clock().now().to_msg(),
            ),
            child_frame_id=self.base_frame_id,
        )
        self.base_t.transform.translation = Vector3(
            x=self.base_pose_position[0, 0], y=self.base_pose_position[0, 1]
        )
        self.base_t.transform.rotation = rotation_quat

        self.base_t.transform = invert_transformation(self.base_t.transform)

        self.tf_broadcaster.sendTransform(self.base_t)

        # States
        self.current_position = self.base_pose_position
        self.current_rotation = self.base_pose_rotation

    def reset_state_service_callback(
        self,
        reset_state_request: ResetState.Request,
        reset_state_response: ResetState.Response,
    ):
        self.reset_state()

        reset_state_response.result = True

        return reset_state_response

    def update_odometry(self, message: Odometry):
        current_pose = message.pose.pose

        self.current_position = np.array(
            [[current_pose.position.x, current_pose.position.y]],
            dtype=np.float64,
        )
        self.current_rotation = quat_message_to_rotation(
            current_pose.orientation
        ).as_matrix()

        current_transform = np.eye(3, 3, dtype=np.float64)
        current_transform[:2, :2] = self.current_rotation[:2, :2]
        current_transform[:2, 2] = np.squeeze(self.current_position)

        base = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 1.0]])

        relative_base = (
            base @ current_transform.T @ self.base_transform_inverse.T
        )

        relative_base_heading = relative_base[1] - relative_base[0]
        relative_base_angle = np.arctan2(
            relative_base_heading[1], relative_base_heading[0]
        )

        rotation_quat = rotation_to_quat_message(
            Rotation.from_euler('z', relative_base_angle)
        )

        new_odometry = Odometry(
            header=Header(
                frame_id=self.base_frame_id,
                stamp=self.get_clock()
                .now()
                .to_msg(),  # message.header.stamp #self.get_clock().now().to_msg()
            ),
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(
                        x=relative_base[0, 0], y=relative_base[0, 1]
                    ),
                    orientation=rotation_quat,
                )
            ),
        )

        current_t_to_base_link = TransformStamped(
            header=Header(
                frame_id=self.base_link_frame_id,
                stamp=self.get_clock().now().to_msg(),
            ),
            child_frame_id=self.odom_frame_id,
        )

        current_t = TransformStamped(
            header=Header(
                frame_id=self.odom_frame_id,
                stamp=self.get_clock()
                .now()
                .to_msg(),  # message.header.stamp #self.get_clock().now().to_msg()
            ),
            child_frame_id=self.base_frame_id,
        )
        current_t.transform.translation = Vector3(
            x=relative_base[0, 0], y=relative_base[0, 1]
        )
        current_t.transform.rotation = rotation_quat

        current_t.transform = invert_transformation(current_t.transform)

        self.base_t.header.stamp = self.get_clock().now().to_msg()

        self.odom_publisher.publish(new_odometry)

        self.tf_broadcaster.sendTransform(current_t_to_base_link)
        self.tf_broadcaster.sendTransform(current_t)
        # self.tf_broadcaster.sendTransform(self.base_t)

        # self.get_logger().info('Pub')


def main(args=None):
    rclpy.init(args=args)

    localizer_node = LocalizerNode()

    # Spin Node
    rclpy.spin(localizer_node)

    # Destroy node
    localizer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
