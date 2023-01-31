import atexit

import numpy as np
import pyaudio
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Int16MultiArray, MultiArrayDimension


class AudioCommandNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        # self.channels_ =
        self.declare_parameters(
            namespace="",
            parameters=[
                ("channels", 1),
                ("frames_per_buffer", 1024),
                ("rate", 16000),
            ],
        )

        self.channels_ = int(self.get_parameter("channels").value)
        self.frames_per_buffer_ = int(self.get_parameter("frames_per_buffer").value)
        self.rate_ = int(self.get_parameter("rate").value)

        self.pyaudio_ = pyaudio.PyAudio()
        self.stream_ = self.pyaudio_.open(
            channels=self.channels_,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.frames_per_buffer_,
            rate=self.rate_,
        )

        self.audio_publisher_ = self.create_publisher(
            Int16MultiArray, "~/audio", qos_profile_system_default
        )

        self.audio_publisher_timer_ = self.create_timer(
            float(self.frames_per_buffer_) / float(self.rate_),
            self.audio_publisher_timer_callback_,
        )

        atexit.register(self.cleanup_)

    def audio_publisher_timer_callback_(self) -> None:
        audio = self.stream_.read(self.frames_per_buffer_)
        audio = np.frombuffer(audio, dtype=np.int16)
        audio_msg = Int16MultiArray()
        audio_msg.data = audio.tolist()
        audio_msg.layout.data_offset = 0
        audio_msg.layout.dim.append(
            MultiArrayDimension(label="audio", size=self.frames_per_buffer_, stride=1)
        )
        self.audio_publisher_.publish(audio_msg)

    def cleanup_(self):
        self.stream_.close()
        self.pyaudio_.terminate()


def main(args=None):
    rclpy.init(args=args)
    audio_command_node = AudioCommandNode("audio_command_node")
    rclpy.spin(audio_command_node)
    audio_command_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
