from launch import LaunchDescription
from launch_ros.actions import Node
from whisper_nodes_launch_mixin import ListenMixin, WhisperMixin


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
    ld.add_action(WhisperMixin.arg_model_name())
    ld.add_action(WhisperMixin.arg_n_threads())
    ld.add_action(WhisperMixin.arg_language())
    ld.add_action(
        WhisperMixin.composable_node_container_whisper(
            composable_node_descriptions=[
                ListenMixin.composable_node_listen(
                    remappings=[("/listen/audio", "/audio_listener/audio")],
                ),
                WhisperMixin.composable_node_whisper(
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
