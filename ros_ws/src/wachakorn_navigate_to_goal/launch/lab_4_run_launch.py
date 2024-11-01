from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='wachakorn_navigate_to_goal',
                executable='localizer',
            ),
            Node(
                package='wachakorn_navigate_to_goal',
                executable='scan_to_point_cloud',
            ),
            Node(
                package='wachakorn_navigate_to_goal',
                executable='too_close_monitor',
            ),
            Node(
                package='wachakorn_navigate_to_goal',
                executable='mapper',
            ),
            Node(
                package='wachakorn_chase_object',
                executable='linear_robot_action',
            ),
            Node(
                package='wachakorn_chase_object',
                executable='rotate_robot_action',
            ),
            Node(
                package='wachakorn_navigate_to_goal',
                executable='path_planner',
            ),
            Node(
                package='wachakorn_navigate_to_goal',
                executable='path_controller',
            ),
        ]
    )
