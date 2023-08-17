from typing import Dict, List

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


class AudioBufferMixin:
    @staticmethod
    def composable_node_audio_buffer(**kwargs) -> ComposableNode:
        return ComposableNode(
            package="whisper",
            plugin="whisper::AudioBufferComponent",
            name="audio_buffer",
            extra_arguments=[{"use_intra_process_comms": True}],
            **kwargs
        )


class WhisperMixin:
    @staticmethod
    def arg_model_name() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="model_name",
            default_value="base.en",
            description="Model name for whisper.cpp. Refer to https://huggingface.co/ggerganov/whisper.cpp.",
            choices=[
                "tiny.en",
                "tiny",
                "base.en",
                "base",
                "small.en",
                "small",
                "medium.en",
                "medium",
                "large-v1",
                "large",
            ],
        )

    @staticmethod
    def arg_n_threads() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="n_threads",
            default_value="1",
            description="Number of threads for whisper.cpp.",
        )

    @staticmethod
    def arg_language() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="language",
            default_value="en",
            description="Language for whisper.cpp.",
            choices=["en", "auto"],
        )

    @staticmethod
    def param_model_name() -> Dict[str, LaunchConfiguration]:
        return {"model_name": LaunchConfiguration("model_name", default="base.en")}

    @staticmethod
    def param_n_threads() -> Dict[str, LaunchConfiguration]:
        return {"n_threads": LaunchConfiguration("n_threads", default="1")}

    @staticmethod
    def param_language() -> Dict[str, LaunchConfiguration]:
        return {"language": LaunchConfiguration("language", default="en")}

    @staticmethod
    def composable_node_whisper(**kwargs) -> ComposableNode:
        return ComposableNode(
            package="whisper",
            plugin="whisper::WhisperComponent",
            name="whisper",
            extra_arguments=[{"use_intra_process_comms": True}],
            **kwargs
        )

    @staticmethod
    def composable_node_container_whisper(
        composable_node_descriptions: List[ComposableNode],
    ) -> ComposableNodeContainer:
        whisper_container = ComposableNodeContainer(
            name="whisper_container",
            package="rclcpp_components",
            namespace="",
            executable="component_container",
            output="screen",
            composable_node_descriptions=composable_node_descriptions,
        )
        return whisper_container
