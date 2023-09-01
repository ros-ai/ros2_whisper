from launch import LaunchDescription
from launch_ros.actions import Node
from whisper_server_launch_mixin import WhisperServerMixin


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
    ld.add_action(WhisperServerMixin.arg_model_name())
    ld.add_action(WhisperServerMixin.arg_n_threads())
    ld.add_action(WhisperServerMixin.arg_language())
    ld.add_action(WhisperServerMixin.arg_batch_capacity())
    ld.add_action(WhisperServerMixin.arg_buffer_capacity())
    ld.add_action(WhisperServerMixin.arg_carry_over_capacity())
    ld.add_action(
        WhisperServerMixin.composable_node_container(
            composable_node_descriptions=[
                WhisperServerMixin.composable_node_inference(
                    parameters=[
                        WhisperServerMixin.param_model_name(),
                        WhisperServerMixin.param_n_threads(),
                        WhisperServerMixin.param_language(),
                        WhisperServerMixin.param_batch_capacity(),
                        WhisperServerMixin.param_buffer_capacity(),
                        WhisperServerMixin.param_carry_over_capacity(),
                    ],
                    remappings=[("/whisper/audio", "/audio_listener/audio")],
                    namespace="whisper",
                ),
            ]
        )
    )
    return ld
