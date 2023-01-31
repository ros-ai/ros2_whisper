import collections

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from scipy import signal
from std_msgs.msg import Float64MultiArray, Int16MultiArray, MultiArrayDimension


class AudioToSpectrumNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.audio_subscriber_ = self.create_subscription(
            Int16MultiArray,
            "/audio_command_node/audio",  # remapping
            self.audio_subscriber_callback_,
            qos_profile_system_default,
        )

        self.audio_deque_ = collections.deque(maxlen=40)  # to be tuned via message

        self.compute_spectrum_timer_ = self.create_timer(
            0.1, self.compute_spectrum_timer_callback_
        )

        self.spectogram_publisher_ = self.create_publisher(
            Float64MultiArray, "~/spectrogram", qos_profile_system_default
        )

        plt.ion()

    def audio_subscriber_callback_(self, audio_msg: Int16MultiArray) -> None:
        audio = audio_msg.data
        self.audio_deque_.append(audio)

    def compute_spectrum_timer_callback_(self) -> None:
        if len(self.audio_deque_) == self.audio_deque_.maxlen:
            audio = np.concatenate(self.audio_deque_)
            spectrogram_msg = self.convert_to_spectrum_(audio)
            self.spectogram_publisher_.publish(spectrogram_msg)

    def convert_to_spectrum_(self, audio: np.ndarray) -> Float64MultiArray:
        # compute spectogram with scipy
        f, t, Sxx = signal.spectrogram(
            audio, fs=44100
        )  # to be read from audio message! create message
        spectrogram_msg = Float64MultiArray()
        spectrogram_msg.data = Sxx.flatten().tolist()
        spectrogram_msg.layout.data_offset = 0
        spectrogram_msg.layout.dim = [
            MultiArrayDimension(label="frequency", size=Sxx.shape[0], stride=1),
            MultiArrayDimension(label="time", size=Sxx.shape[1], stride=1),
        ]

        return spectrogram_msg


def main(args=None):
    rclpy.init(args=args)
    audio_spectrum_node = AudioToSpectrumNode("audio_to_spectrum_node")
    rclpy.spin(audio_spectrum_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
