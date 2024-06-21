import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # launch audio listener
    ld.add_action(
        Node(
            package="audio_listener",
            executable="audio_listener",
            output="screen",
        )
    )

    # launch whisper
    whisper_config = os.path.join(
        get_package_share_directory("whisper_server"), "config", "whisper.yaml"
    )
    ld.add_action(
        Node(
            package="whisper_server",
            executable="whisper",
            output="screen",
            namespace="whisper",
            parameters=[whisper_config],
            remappings=[
                ("/whisper/audio", "/audio_listener/audio"),
            ],
        )
    )

    return ld
