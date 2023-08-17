from launch import LaunchDescription

from whisper_launch_mixin import WhisperMixin, AudioBufferMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(WhisperMixin.arg_model_name())
    ld.add_action(WhisperMixin.arg_n_threads())
    ld.add_action(WhisperMixin.arg_language())
    ld.add_action(
        WhisperMixin.composable_node_container_whisper(
            composable_node_descriptions=[
                AudioBufferMixin.composable_node_audio_buffer(),
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
