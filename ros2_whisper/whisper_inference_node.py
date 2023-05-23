from collections import deque

import numpy as np
import rclpy
import torch
import whisper
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Int16MultiArray, String


class WhisperInferenceNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.declare_parameter("model", "base")
        self.declare_parameter("language", "english")
        model = self.get_parameter("model").value
        language = self.get_parameter("language").value

        self.whisper_model_ = whisper.load_model(model)
        self.whisper_options_ = whisper.DecodingOptions(language=language)

        self.declare_parameters(
            namespace="",
            parameters=[("audio_activation_threshold", 0.3), ("inference_period", 1.0)],
        )
        self.audio_activation_threshold_ = (
            self.get_parameter("audio_activation_threshold")
            .get_parameter_value()
            .double_value
        )
        self.inference_period_ = (
            self.get_parameter("inference_period").get_parameter_value().double_value
        )

        self.audio_subscriber_ = self.create_subscription(
            Int16MultiArray,
            "/audio_listener_node/audio",
            self.audio_subscriber_callback_,
            qos_profile_system_default,
        )
        self.audio_buffer_ = deque(
            maxlen=int(16000.0 / 1024 * self.inference_period_)
        )  # buffer length to record self.inference_period_ seconds

        self.whisper_timer_ = self.create_timer(
            1.0 / self.inference_period_, self.whisper_timer_callback_
        )

        self.text_publisher_ = self.create_publisher(
            String, "~/text", qos_profile_system_default
        )

        self.device_ = "cpu"
        if torch.cuda.is_available():
            self.get_logger().info("CUDA is available. Using GPU.")
            self.device_ = "cuda"
        self.whisper_model_ = self.whisper_model_.to(self.device_)

    def audio_subscriber_callback_(self, audio_msg: Int16MultiArray) -> None:
        self.audio_buffer_.append(audio_msg.data)

    def whisper_timer_callback_(self):
        if len(self.audio_buffer_) == self.audio_buffer_.maxlen:
            audio = (
                np.concatenate(self.audio_buffer_) / 32768.0
            )  # normalization in whisper https://github.com/openai/whisper/blob/0f39c89d9212e4d0c64b915cf7ba3c1f0b59fecc/whisper/audio.py#L49
            audio = torch.from_numpy(audio).float()
            if audio.abs().max() < self.audio_activation_threshold_:
                return
            audio = whisper.pad_or_trim(audio)
            mel = whisper.log_mel_spectrogram(audio).to(self.device_)
            result = whisper.decode(self.whisper_model_, mel, self.whisper_options_)

            text_msg = String(data=result.text)
            self.text_publisher_.publish(text_msg)


def main(args=None):
    rclpy.init(args=args)
    whisper_inference_node = WhisperInferenceNode("whisper_inference_node")
    rclpy.spin(whisper_inference_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
