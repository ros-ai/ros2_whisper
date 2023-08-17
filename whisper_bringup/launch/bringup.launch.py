from launch import LaunchDescription
from launch_ros.actions import Node
from whisper_launch_mixin import AudioBufferMixin, WhisperMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # launch audio listener
    ld.add_action(
        Node(
            package="audio_listener",
            executable="audio_listener_node",
            output="screen",
        )
    )

    # launch whisper
    ld.add_action(WhisperMixin.arg_model_name())
    ld.add_action(WhisperMixin.arg_n_threads())
    ld.add_action(WhisperMixin.arg_language())
    ld.add_action(
        WhisperMixin.composable_node_container_whisper(
            composable_node_descriptions=[
                AudioBufferMixin.composable_node_audio_buffer(
                    remappings=[("/audio_buffer/audio", "/audio_listener_node/audio")],
                ),
                WhisperMixin.composable_node_whisper(
                    remappings=[
                        ("/whisper/provide", "/audio_buffer/provide"),
                    ],
                    parameters=[
                        WhisperMixin.param_model_name(),
                        WhisperMixin.param_n_threads(),
                        WhisperMixin.param_language(),
                    ],
                ),
            ]
        )
    )
    return ld
