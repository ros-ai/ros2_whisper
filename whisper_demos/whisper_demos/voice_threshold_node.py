import array

import numpy as np
import rclpy
from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_msgs.msg import Int16MultiArray
from whisper_msgs.action import Listen
from whisper_msgs.srv import Inference


class VoiceThresholdNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name=node_name)

        if not self.has_parameter("threshold"):
            self.declare_parameter("threshold", 0.2)

        # listen
        self.listen_client = ActionClient(
            self, Listen, "/whisper/listen"
        )  # TODO: replace this with a proper name
        while not self.listen_client.wait_for_server(1):
            self.get_logger().warn(
                f"Waiting for {self.listen_client._action_name} action server.."
            )
        self.get_logger().info(
            f"Action server {self.listen_client._action_name} found."
        )

        # inference
        self.inference_client = self.create_client(
            Inference, "/whisper/inference"
        )  # TODO: replace
        while not self.inference_client.wait_for_service(1):
            self.get_logger().warn(
                f"Waiting for {self.inference_client.srv_name} service.."
            )
        self.get_logger().info(f"Service {self.inference_client.srv_name} found.")

        self.audio_subscription = self.create_subscription(
            Int16MultiArray,
            # "~/audio", # TODO: replace node name
            "/audio_listener/audio",
            self.on_audio_,
            10,
        )

    def normalize_(self, audio: array) -> np.ndarray:
        return np.array(audio).astype(float) / 32768.0

    def on_audio_(self, audio: Int16MultiArray) -> None:
        audio_normalized = self.normalize_(audio.data)

        if np.max(audio_normalized) > self.get_parameter("threshold").value:
            self.get_logger().info("Voice detected")
            goal = Listen.Goal()
            goal.max_duration.sec = 5
            self.goal_future_ = self.listen_client.send_goal_async(goal)
            self.goal_future_.add_done_callback(self.on_listen_goal_)

    def on_listen_goal_(self, goal_future: Future):
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            return
        self.get_logger().info("Goal accepted.")

        self.result_future_ = goal_handle.get_result_async()
        self.result_future_.add_done_callback(self.on_listen_result_)

    def on_listen_result_(self, result_future: Future):
        result: Listen.Result = result_future.result().result

        # run inference
        requset = Inference.Request()
        requset.audio = result.audio
        requset.n_processors = 1
        self.service_future = self.inference_client.call_async(requset)
        self.service_future.add_done_callback(self.on_inference_result_)

    def on_inference_result_(self, result_future: Future):
        response: Inference.Response = result_future.result()
        self.get_logger().info(f"Response: {response.info}")
        self.get_logger().info(f"Segments: {len(response.segments)}")
        for segment in response.segments:
            self.get_logger().info(f"Segment: {segment}")


def main(args=None):
    rclpy.init(args=args)
    voice_threshold_activation = VoiceThresholdNode(
        node_name="voice_threshold_activation"
    )
    rclpy.spin(voice_threshold_activation)
    rclpy.shutdown()
