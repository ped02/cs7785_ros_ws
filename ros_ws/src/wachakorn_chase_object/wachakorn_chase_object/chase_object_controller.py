from collections.abc import Mapping, Callable, Collection

import numpy as np

import rclpy
import rclpy.time
from rclpy.action import ActionClient
from rclpy.node import (
    Node,
    Parameter,
    ParameterDescriptor,
    SetParametersResult,
    MutuallyExclusiveCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PointStamped
from robot_interfaces.action import Control

from .parameter_utils import validate_positive_double


def signed_angle_difference(vector_a, vector_b):
    """Vector b is target vector."""
    x = np.sum(np.multiply(vector_a, vector_b))
    y = np.cross(vector_a, vector_b)
    return np.arctan2(y, x)


def get_target_point(object_vector, offset_distance_meter, EPSILON=1e-6):
    object_point = object_vector[0, :-1]  # Dont care about z

    target_opposite_vector = -object_point

    target_opposite_vector_norm = np.linalg.norm(target_opposite_vector)

    if abs(target_opposite_vector_norm) < EPSILON:
        # Assume zero vector, not accurate info
        return

    target_opposite_vector_unit = (
        target_opposite_vector / target_opposite_vector_norm
    )

    target_offset_vector = offset_distance_meter * target_opposite_vector_unit

    # Get vector to x distance away from object
    return object_point + target_offset_vector


class ChaseObjectController(Node):
    def __init__(self, **kwargs):
        super(ChaseObjectController, self).__init__(
            'chase_object_controller', **kwargs
        )

        # Parameters
        parameter_validator: Mapping[
            str, Callable[[Parameter], SetParametersResult]
        ] = {}

        target_distance_meter_parameter_descriptor = ParameterDescriptor(
            description='Target distance in meters.'
        )
        self.declare_parameter(
            'target_distance_meter',
            0.5,
            target_distance_meter_parameter_descriptor,
        )
        parameter_validator['target_distance_meter'] = validate_positive_double

        target_distance_deadzone_meter_parameter_descriptor = (
            ParameterDescriptor(
                description='Target distance deadzone in meters.'
            )
        )
        self.declare_parameter(
            'target_distance_deadzone_meter',
            0.1,
            target_distance_deadzone_meter_parameter_descriptor,
        )
        parameter_validator[
            'target_distance_deadzone_meter'
        ] = validate_positive_double

        target_stale_duration_sec_parameter_descriptor = ParameterDescriptor(
            description='Target stale in seconds.'
        )
        self.declare_parameter(
            'target_stale_duration_sec',
            0.5,
            target_stale_duration_sec_parameter_descriptor,
        )
        parameter_validator[
            'target_stale_duration_sec'
        ] = validate_positive_double

        # target_angle_deadzone_radians_parameter_descriptor = ParameterDescriptor(description='Target distance deadzone in meters.')
        # self.declare_parameter('target_angle_deadzone_radians')

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

        # Transforms
        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer, self)

        # Get Target Distance and Angle
        state_update_callback_group = MutuallyExclusiveCallbackGroup()

        self.reset()
        self.target_point_subscription = self.create_subscription(
            PointStamped,
            '/target_object',
            self.set_target,
            10,
            callback_group=state_update_callback_group,
        )

        # Actions
        rotate_action_callback_group = MutuallyExclusiveCallbackGroup()
        self.rotate_action_client = ActionClient(
            self,
            Control,
            '/rotate_robot',
            callback_group=rotate_action_callback_group,
        )

        linear_action_callback_group = MutuallyExclusiveCallbackGroup()
        self.move_action_client = ActionClient(
            self,
            Control,
            '/move_robot',
            callback_group=linear_action_callback_group,
        )

        while not self.rotate_action_client.wait_for_server(0.5):
            self.get_logger().info('Waiting for rotate action server')

        while not self.move_action_client.wait_for_server(0.5):
            self.get_logger().info('Waiting for move action server')

        # Execute
        timer_callback_group = MutuallyExclusiveCallbackGroup()
        EXECUTE_POLL_PERIOD = 0.9
        self.execute_timer = self.create_timer(
            EXECUTE_POLL_PERIOD, self.chase, callback_group=timer_callback_group
        )

    def update_parameters(self):
        self.target_distance_meter = (
            self.get_parameter('target_distance_meter')
            .get_parameter_value()
            .double_value
        )
        self.target_distance_deadzone_meter = (
            self.get_parameter('target_distance_deadzone_meter')
            .get_parameter_value()
            .double_value
        )

        self.target_stale_duration_sec = (
            self.get_parameter('target_stale_duration_sec')
            .get_parameter_value()
            .double_value
        )

    def set_target(self, message: PointStamped):
        self.target_frame_id = message.header.frame_id
        self.target_timestamp = rclpy.time.Time.from_msg(message.header.stamp)
        self.object_point = np.array(
            [[message.point.x, message.point.y, message.point.z]],
            dtype=np.float32,
        )

    def reset(self):
        self.target_frame_id = ''
        self.target_timestamp = rclpy.time.Time(
            clock_type=self.get_clock().clock_type
        )
        self.object_point = np.zeros((1, 3), dtype=np.float32)

    async def chase(self):
        self.update_parameters()

        # Calculate Target Position
        if (
            abs(
                (self.get_clock().now() - self.target_timestamp).nanoseconds
                * 1e-9
            )
            > self.target_stale_duration_sec
        ):
            # Data stale
            return

        # (Assume fixed frame rotation)
        # target_frame_id = 'base_link'
        # frame_transform = None
        # try:
        #     frame_transform = self.transform_buffer.lookup_transform(
        #         target_frame_id,
        #         self.target_frame_id,
        #         self.get_clock().now().to_msg()
        #     )
        # except TransformException as ex:
        #     self.get_logger().info(f'Could not transform \'{self.target_frame_id}\' to \'{target_frame_id}\': {ex}')

        # if frame_transform is None:
        #     # Fail: wait to try again
        #     return

        # frame_rotation = quat_message_to_rotation(frame_transform.transform.rotation)

        # frame_transform_matrix = np.eye(4,4, dtype=np.float32)
        # frame_transform_matrix[:3, :3] = frame_rotation.as_matrix()
        # frame_transform_matrix[:3, 3] = np.array([frame_transform.transform.translation.x,
        #                                           frame_transform.transform.translation.y,
        #                                           frame_transform.transform.translation.z])

        # Get vector to x distance away from object
        x = np.array([1.0, 0.0])
        target_vector = get_target_point(
            self.object_point, self.target_distance_meter
        )

        if target_vector is None:
            # invalid vector
            return

        # self.get_logger().info(f'Object: {self.object_point}. Target Vector: {}')

        # angle_difference = np.sin(np.cross(target_vector, x) / (np.linalg.norm(target_vector))) # Someone points behind will cause rotation to the other side (wrong side)
        # signed_angle_difference(target_vector, x) points at back is behind so it will rotate backward

        angle_difference_1 = signed_angle_difference(x, target_vector)
        angle_difference_2 = signed_angle_difference(x, -target_vector)

        angle_difference = min(angle_difference_1, angle_difference_2, key=abs)

        ROTATION_THRESHOLD_RADIANS = 10 / 180 * np.pi

        ROTATE_TIMEOUT_SEC = 0.5
        MOVE_TIMEOUT_SEC = 0.5

        sleep_rate = self.create_rate(1 / 0.05)

        # self.get_logger().info(f'Angle Difference: {angle_difference * 180 / np.pi}')
        if abs(angle_difference) > ROTATION_THRESHOLD_RADIANS:
            # Rototate to match
            rotation_control_goal = Control.Goal()
            rotation_control_goal.target_value = angle_difference
            rotate_control_future = self.rotate_action_client.send_goal_async(
                rotation_control_goal
            )

            while not rotate_control_future.done():
                sleep_rate.sleep()

            rotate_goal_handle = rotate_control_future.result()

            rotate_result_future = rotate_goal_handle.get_result_async()

            control_start_time = self.get_clock().now()
            while (
                (self.get_clock().now() - control_start_time).nanoseconds * 1e-9
            ) < ROTATE_TIMEOUT_SEC:
                if rotate_result_future.done():
                    break
                sleep_rate.sleep()

            if not rotate_result_future.done():
                self.rotate_action_client._cancel_goal(rotate_goal_handle)
                # self.get_logger().info(f'Cancelled rotate')
                return

        # Update target vector after rotation
        target_vector = get_target_point(
            self.object_point, self.target_distance_meter
        )
        distance_difference = np.sum(np.multiply(target_vector, x))

        if abs(distance_difference) > self.target_distance_deadzone_meter:
            linear_control_goal = Control.Goal()
            linear_control_goal.target_value = distance_difference
            linear_control_future = self.move_action_client.send_goal_async(
                linear_control_goal
            )

            while not linear_control_future.done():
                sleep_rate.sleep()

            linear_goal_handle = linear_control_future.result()

            linear_result_future = linear_goal_handle.get_result_async()

            control_start_time = self.get_clock().now()

            while (
                (self.get_clock().now() - control_start_time).nanoseconds * 1e-9
            ) < MOVE_TIMEOUT_SEC:
                if linear_result_future.done():
                    break
                sleep_rate.sleep()

            if not linear_result_future.done():
                self.move_action_client._cancel_goal(linear_goal_handle)
                # self.get_logger().info(f'Cancelled linear')
                return

        # Target reached
        # self.get_logger().info(f'Target reached: {target_vector} - {distance_difference}m {angle_difference * 180 / np.pi}deg')


def main(args=None):
    rclpy.init(args=args)

    chase_object_controller = ChaseObjectController()

    executor = MultiThreadedExecutor()
    executor.add_node(chase_object_controller)

    # Spin Node
    executor.spin()

    # Destroy node
    chase_object_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
