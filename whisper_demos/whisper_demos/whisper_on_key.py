import rclpy
from builtin_interfaces.msg import Duration
from pynput.keyboard import Key, Listener
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Int16MultiArray
from whisper_msgs.action import Inference


class WhisperOnKey(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name=node_name)

        # whisper
        self.whisper_client = ActionClient(self, Inference, "/whisper/listen")

        while not self.whisper_client.wait_for_server(1):
            self.get_logger().warn(
                f"Waiting for {self.whisper_client._action_name} action server.."
            )
        self.get_logger().info(
            f"Action server {self.whisper_client._action_name} found."
        )

        self.key_listener = Listener(on_press=self.on_key)
        self.key_listener.start()

        self.get_logger().info(self.info())

    def on_key(self, key) -> None:
        if key == Key.esc:
            self.key_listener.stop()
            rclpy.shutdown()
            return

        if key == Key.space:
            # inference goal
            return

        self.get_logger().warn("Invalid key")

    def info(self) -> str:
        return (
            "\n"
            "\tStarting Whisper on Key demo.\n"
            "\tPress ESC to exit.\n"
            "\tPress space to start listening.\n"
            "\tPress space again to stop listening.\n"
        )


def main(args=None):
    rclpy.init(args=args)
    whisper_on_key = WhisperOnKey(node_name="whisper_on_key")
    rclpy.spin(whisper_on_key)
