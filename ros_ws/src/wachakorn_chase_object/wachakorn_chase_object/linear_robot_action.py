from collections.abc import Collection, Mapping
from typing import Callable

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import (
    Node,
    Parameter,
    ParameterDescriptor,
    SetParametersResult,
    MutuallyExclusiveCallbackGroup,
)
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

from robot_interfaces.srv import ResetState
from robot_interfaces.action import Control

from .parameter_utils import validate_positive_double
from .ros_utils import ros_time_to_seconds

from .controller import Controller

# from .bang_bang_controller import BangBangController
from .clamped_proportional_controller import ClampedProportionalController


def radians_to_vector(radius, theta):
    return radius * np.array([np.cos(theta), np.sin(theta)])


def signed_angle_difference(vector_a, vector_b):
    """Vector b is target vector."""
    x = np.sum(np.multiply(vector_a, vector_b))
    y = np.cross(vector_a, vector_b)
    return np.arctan2(y, x)


class LinearMoveActionServer(Node):
    def __init__(self, **kwargs):
        super(LinearMoveActionServer, self).__init__(
            'linear_robot_action', **kwargs
        )

        parameter_validator: Mapping[
            str, Callable[[Parameter], SetParametersResult]
        ] = {}

        odometry_max_period_sec_parameter_descriptor = ParameterDescriptor(
            description='Max period (time between next) to count odometry as valid in sec'
        )
        self.declare_parameter(
            'odometry_max_period_sec',
            0.1,
            odometry_max_period_sec_parameter_descriptor,
        )

        parameter_validator[
            'odometry_max_period_sec'
        ] = validate_positive_double

        distance_deadzone_meter_parameter_descriptor = ParameterDescriptor(
            description='Distance deadzone in meters'
        )
        self.declare_parameter(
            'distance_deadzone_meter',
            0.05,
            distance_deadzone_meter_parameter_descriptor,
        )

        parameter_validator[
            'distance_deadzone_meter'
        ] = validate_positive_double

        update_parameters_period_sec_parameter_descriptor = ParameterDescriptor(
            description='Update parameter period in seconds'
        )
        self.declare_parameter(
            'update_parameters_period_sec',
            0.02,
            update_parameters_period_sec_parameter_descriptor,
        )

        parameter_validator[
            'update_parameters_period_sec'
        ] = validate_positive_double

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

        self.update_parameter_callback_group = MutuallyExclusiveCallbackGroup()
        UPDATE_PARAMETER_POLL_PERIOD_SEC = 0.005
        self.update_parameters_timer = self.create_timer(
            UPDATE_PARAMETER_POLL_PERIOD_SEC,
            self.update_parameters,
            callback_group=self.update_parameter_callback_group,
        )

        self.reset_state()
        self.running = False

        # State Updates
        odometry_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=3,
        )

        self.state_callback_group = MutuallyExclusiveCallbackGroup()
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.update_state_odometry,
            odometry_qos_profile,
            callback_group=self.state_callback_group,
        )

        self.reset_state_service = self.create_service(
            ResetState,
            '/_reset_state',
            self.reset_state_service_callback,
            callback_group=self.state_callback_group,
        )

        self.reset_state_callback_group = MutuallyExclusiveCallbackGroup()
        self.reset_state_client = self.create_client(
            ResetState,
            '/_reset_state',
            callback_group=self.reset_state_callback_group,
        )

        # Action
        self.action_callback_group = MutuallyExclusiveCallbackGroup()
        self.action_server = ActionServer(
            self,
            Control,
            'move_robot',
            execute_callback=self.execute_action_callback,
            callback_group=self.action_callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Publish
        command_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=3,
        )
        self.command_publisher = self.create_publisher(
            Twist, '/cmd_vel', command_qos_profile
        )

    def update_parameters(self):
        self.odometry_max_period_sec = (
            self.get_parameter('odometry_max_period_sec')
            .get_parameter_value()
            .double_value
        )
        self.distance_deadzone_meter = (
            self.get_parameter('distance_deadzone_meter')
            .get_parameter_value()
            .double_value
        )

        self.parameter_update_period_sec = (
            self.get_parameter('update_parameters_period_sec')
            .get_parameter_value()
            .double_value
        )

    def reset_state(self):
        self.current_vector = np.zeros(3, dtype=np.float32)
        self.last_odom_vector = np.zeros(3, dtype=np.float32)

        self.last_odometry_time = 0.0  # No last odometry

    def update_state_odometry(self, odometry: Odometry):
        odometry_time_sec = ros_time_to_seconds(self.get_clock().now())

        odom_vector = np.array(
            [odometry.pose.pose.position.x, odometry.pose.pose.position.y, 0.0],
            dtype=np.float32,
        )
        odom_orientation = Rotation.from_quat(
            [
                odometry.pose.pose.orientation.x,
                odometry.pose.pose.orientation.y,
                odometry.pose.pose.orientation.z,
                odometry.pose.pose.orientation.w,
            ]
        ).as_matrix()

        dt = odometry_time_sec - self.last_odometry_time
        if dt <= self.odometry_max_period_sec:
            # Take this odometry as valid
            # self.current_theta += angular_z * dt # Some how this got problem
            self.current_vector += odom_orientation.T @ (
                odom_vector - self.last_odom_vector
            )
            # self.get_logger().info(f'[{dt}] {odom_vector - self.last_odom_vector}')

        self.last_odometry_time = odometry_time_sec
        self.last_odom_vector = odom_vector

        # self.get_logger().info('Received odometry')

    def reset_state_service_callback(
        self,
        reset_state_request: ResetState.Request,
        reset_state_response: ResetState.Response,
    ):
        self.reset_state()

        reset_state_response.result = True

        return reset_state_response

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')

        if self.running:
            return GoalResponse.REJECT

        reset_state_response: ResetState.Response = (
            self.reset_state_client.call(ResetState.Request())
        )
        if not reset_state_response.result:
            # Reset state failed
            self.get_logger().error('[Goal Callback] Reset State failed!')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, cancel_request):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')

        if not self.running:
            return CancelResponse.REJECT

        # Stop robot
        self.running = False
        self.stop_robot()
        self.get_logger().info('Stopping Robot')

        return CancelResponse.ACCEPT

    def stop_robot(self):
        self.command_publisher.publish(Twist(linear=Vector3(x=0.0)))

    def execute_action_callback(self, goal_handle: ServerGoalHandle):
        self.running = True

        forward_axis = np.array([1, 0, 0], dtype=np.float32)

        goal_request: Control.Goal = goal_handle.request
        target_displacement_meters = goal_request.target_value

        feedback_message = Control.Feedback()

        # Controller
        BURGER_MAX_VELOCITY = 0.22  # m/s

        CONTROLLER_COMMAND_MIN = BURGER_MAX_VELOCITY * 0.2
        # CONTROLLER_COMMAND_MAX = BURGER_MAX_VELOCITY * 0.8
        CONTROLLER_COMMAND_MAX = BURGER_MAX_VELOCITY * 0.4

        kp = 1 / 0.5  # give 1/0.5 seconds to correct

        controller: Controller = ClampedProportionalController(
            kp=kp,
            command_min=CONTROLLER_COMMAND_MIN,
            command_max=CONTROLLER_COMMAND_MAX,
        )

        previous_time = self.get_clock().now()
        while self.running:
            current_vector = self.current_vector
            current_time = self.get_clock().now()

            error_distance_meter = float(
                target_displacement_meters
                - np.sum(np.multiply(current_vector, forward_axis))
            )

            if abs(error_distance_meter) < self.distance_deadzone_meter:
                # Goal Reached
                self.running = False
                break

            # Command
            twist_command = Twist()
            twist_command.linear.x = controller.execute(
                error_distance_meter, current_time, previous_time
            )
            self.command_publisher.publish(twist_command)

            # Feedback
            feedback_message.error_value = error_distance_meter
            goal_handle.publish_feedback(feedback_message)

            previous_time = current_time

        self.stop_robot()
        self.running = False

        goal_handle.succeed()

        result = Control.Result()
        result.final_value = float(
            np.sum(np.multiply(current_vector, forward_axis))
        )
        result.error_value = float(
            target_displacement_meters
            - np.sum(np.multiply(current_vector, forward_axis))
        )

        print(result)

        return result


def main(args=None):
    rclpy.init(args=args)

    linear_move_action_server = LinearMoveActionServer()

    executor = MultiThreadedExecutor(4)
    executor.add_node(linear_move_action_server)

    # Spin Node
    executor.spin()

    # Destroy node
    linear_move_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
