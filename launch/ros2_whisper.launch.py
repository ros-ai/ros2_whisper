from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart


def generate_launch_description():

    audio_listener_node = Node(
        package="ros2_whisper",
        executable="audio_listener_node",
        output="screen",
    )

    whisper_inference_node = Node(
        package="ros2_whisper",
        executable="whisper_inference_node",
        output="screen",
    )

    whisper_inference_node_event_handler = RegisterEventHandler(
        OnProcessStart(
            on_start=[whisper_inference_node], target_action=audio_listener_node
        )
    )

    return LaunchDescription([audio_listener_node, whisper_inference_node_event_handler])
