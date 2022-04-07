import launch
from launch_ros.actions import Node


def generate_launch_description():
    logger = launch.substitutions.LaunchConfiguration("log_level")
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        ),
        Node(
            package='omniwheel',
            executable='path_vis',
            name='path_vis',
            arguments=['--ros-args', '--log-level', logger]
        ),
        Node(
            package='omniwheel',
            executable='path_exec',
            name='path_exec',
            arguments=['--ros-args', '--log-level', logger]
        ),
        Node(
            package='omniwheel',
            executable='frame_publisher',
            name='frame_publisher',
            arguments=['--ros-args', '--log-level', logger]
        )
    ])