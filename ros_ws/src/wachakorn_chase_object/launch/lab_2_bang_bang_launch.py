from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='wachakorn_chase_object',
                executable='track_color_headless',
            ),
            Node(
                package='wachakorn_chase_object',
                executable='rotate_robot_bang_bang',
            ),
        ]
    )
