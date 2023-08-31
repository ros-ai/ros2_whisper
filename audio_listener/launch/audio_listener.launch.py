from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(
        Node(
            package="audio_listener",
            executable="audio_listener",
            output="screen",
        )
    )
    return ld
