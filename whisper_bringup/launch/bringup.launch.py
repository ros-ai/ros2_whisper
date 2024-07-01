import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


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
    composable_node = ComposableNode(
        package="whisper_server",
        plugin="whisper::InferenceComponent",
        name="inference",
        namespace="whisper",
        parameters=[whisper_config],
        remappings=[("audio", "/audio_listener/audio")],
    )
    ld.add_action(
        ComposableNodeContainer(
            name="whisper_container",
            package="rclcpp_components",
            namespace="",
            executable="component_container_mt",  # require multi-threaded executor so inference server can parallelize audio encueing and inference
            output="screen",
            composable_node_descriptions=[composable_node],
        )
    )
    return ld
