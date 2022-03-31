from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='omniwheel',
            executable='path_vis',
            name='path_vis'
        ),
        Node(
            package='omniwheel',
            executable='path_exec',
            name='path_exec'
        ),
        Node(
            package='omniwheel',
            executable='frame_publisher',
            name='frame_publisher'
        )
    ])