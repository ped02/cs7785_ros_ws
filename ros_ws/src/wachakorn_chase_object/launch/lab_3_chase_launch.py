from launch import LaunchDescription
from launch_ros.actions import Node

debug_visualization = True


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='wachakorn_chase_object',
                executable='track_color_headless',
                parameters=[{'display_tracking': debug_visualization}],
            ),
            Node(
                package='wachakorn_chase_object',
                executable='object_bounding_to_plane',
                parameters=[
                    {
                        'target_frame_id': 'base_scan',
                        'display_plane': debug_visualization,
                    }
                ],
            ),
            Node(
                package='wachakorn_chase_object',
                executable='filter_scan_points',
                parameters=[
                    {
                        'target_frame_id': 'base_scan',
                        'max_sync_time_diff_sec': '30',
                    }
                ],
            ),
            Node(
                package='wachakorn_chase_object',
                executable='object_localize_filter',
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
                package='wachakorn_chase_object',
                executable='chase_object_controller',
            ),
        ]
    )
