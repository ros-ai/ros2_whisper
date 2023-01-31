import torch.nn.functional as F
import numpy as np
import torch
import whisper
from collections import deque
import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Float32MultiArray, Int16MultiArray


class WhisperInferenceNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.whisper_model_ = whisper.load_model("base")
        self.whisper_options_ = whisper.DecodingOptions()

        self.audio_subscriber_ = self.create_subscription(
            Int16MultiArray,
            "/audio_command_node/audio",
            self.audio_subscriber_callback_,
            qos_profile_system_default,
        )
        self.period_ = 1.0
        self.audio_buffer_ = deque(
            maxlen=int(16000.0 / 1024 * self.period_)
        )  # buffer length to record 1. seconds

        self.whisper_timer_ = self.create_timer(
            1.0 / self.period_, self.whisper_timer_callback_
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
            )  # normalization in whisper
            audio = torch.from_numpy(audio).float()
            audio = whisper.pad_or_trim(audio)
            mel = whisper.log_mel_spectrogram(audio).to(self.device_)
            result = whisper.decode(self.whisper_model_, mel, self.whisper_options_)

            self.get_logger().info(result.text)


def main(args=None):
    rclpy.init(args=args)
    whisper_inference_node = WhisperInferenceNode("whisper_inference_node")
    rclpy.spin(whisper_inference_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
