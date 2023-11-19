from typing import Dict, List

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


class InferenceMixin:
    @staticmethod
    def arg_model_name() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="model_name",
            default_value="tiny.en",
            description="Model name for whisper.cpp. Refer to https://huggingface.co/ggerganov/whisper.cpp.",
            choices=[
                "tiny.en",
                "tiny",
                "tiny.en",
                "base",
                "base.en",
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
            default_value="4",
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
    def arg_batch_capacity() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="batch_capacity",
            default_value="6",
            description="Batch capacity in seconds.",
        )

    @staticmethod
    def arg_buffer_capacity() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="buffer_capacity",
            default_value="2",
            description="Buffer capacity in seconds.",
        )

    @staticmethod
    def arg_carry_over_capacity() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="carry_over_capacity",
            default_value="200",
            description="Carry over capacity in milliseconds.",
        )

    @staticmethod
    def param_model_name() -> Dict[str, LaunchConfiguration]:
        return {"model_name": LaunchConfiguration("model_name", default="tiny.en")}

    @staticmethod
    def param_n_threads() -> Dict[str, LaunchConfiguration]:
        return {"n_threads": LaunchConfiguration("n_threads", default="4")}

    @staticmethod
    def param_language() -> Dict[str, LaunchConfiguration]:
        return {"language": LaunchConfiguration("language", default="en")}

    @staticmethod
    def param_batch_capacity() -> Dict[str, LaunchConfiguration]:
        return {"batch_capacity": LaunchConfiguration("batch_capacity", default="6")}

    @staticmethod
    def param_buffer_capacity() -> Dict[str, LaunchConfiguration]:
        return {"buffer_capacity": LaunchConfiguration("buffer_capacity", default="2")}

    @staticmethod
    def param_carry_over_capacity() -> Dict[str, LaunchConfiguration]:
        return {
            "carry_over_capacity": LaunchConfiguration(
                "carry_over_capacity", default="200"
            )
        }

    @staticmethod
    def composable_node_inference(**kwargs) -> ComposableNode:
        return ComposableNode(
            package="whisper_server",
            plugin="whisper::InferenceComponent",
            name="inference",
            extra_arguments=[{"use_intra_process_comms": True}],
            **kwargs
        )


class WhisperServerMixin(InferenceMixin):
    @staticmethod
    def composable_node_container(
        composable_node_descriptions: List[ComposableNode],
    ) -> ComposableNodeContainer:
        whisper_container = ComposableNodeContainer(
            name="whisper_container",
            package="rclcpp_components",
            namespace="",
            executable="component_container_mt",  # TODO: add note why this is important
            output="screen",
            composable_node_descriptions=composable_node_descriptions,
        )
        return whisper_container
