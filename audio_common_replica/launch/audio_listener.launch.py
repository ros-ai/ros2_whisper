from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(
        Node(
            package="audio_common_replica",
            executable="audio_listener_node",
            output="screen",
        )
    )
    return ld
