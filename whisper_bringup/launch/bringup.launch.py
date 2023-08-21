from launch import LaunchDescription
from launch_ros.actions import Node
from whisper_nodes_launch_mixin import WhisperNodesMixin


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
    ld.add_action(WhisperNodesMixin.arg_model_name())
    ld.add_action(WhisperNodesMixin.arg_n_threads())
    ld.add_action(WhisperNodesMixin.arg_language())
    ld.add_action(
        WhisperNodesMixin.composable_node_container(
            composable_node_descriptions=[
                WhisperNodesMixin.composable_node_inference(
                    parameters=[
                        WhisperNodesMixin.param_model_name(),
                        WhisperNodesMixin.param_n_threads(),
                        WhisperNodesMixin.param_language(),
                    ],
                    remappings=[("/whisper/audio", "/audio_listener/audio")],
                    namespace="whisper",
                ),
            ]
        )
    )
    return ld
