import rclpy
from rclpy.node import Node
from whisper_idl.msg import AudioTranscript
import os
from datetime import datetime, timedelta
from builtin_interfaces.msg import Time

def time_to_string(timestamp : Time) -> str:
    dt = datetime.fromtimestamp(timestamp.sec) + timedelta(microseconds=timestamp.nanosec / 1000)
    formatted_time = dt.strftime("%Y-%m-%d %H-%M-%S.%f")[:-3]  # Truncate to milliseconds
    return formatted_time

def color_gradient(word, value, min_value, max_value):
    """
    Outputs a value as colored text with a gradient from red to green based on proximity to the max threshold.
    
    Args:
        value (float): The numeric value to evaluate.
        min_value (float): The lower bound of the range.
        max_value (float): The upper bound of the range (threshold).
    
    Returns:
        str: Colored text gradually changing from red to green as value approaches max_value.
    """
    # Normalize value to a 0-1 range
    proportion = (value - min_value) / (max_value - min_value)
    proportion = max(0, min(1, proportion))  # Clamp proportion between 0 and 1
    
    # Calculate the RGB values (red to green gradient)
    red = int((1 - proportion) * 255)       # Decreases from 255 to 0 as value approaches max
    green = int(proportion * 255)           # Increases from 0 to 255 as value approaches max
    
    # ANSI escape code for RGB colors: \033[38;2;<R>;<G>;<B>m
    colored_text = f"\033[38;2;{red};{green};0m{word}\033[0m"
    
    return colored_text

class TranscriptSubscriber(Node):

    def __init__(self):
        super().__init__('transcript_subscriber')

        # Control what elements of the transcript are displayed
        self.declare_parameter('threshold', 2.0)
        
        # Subscribe to the audio transcript topic
        self.subscription = self.create_subscription(
            AudioTranscript,
            '/whisper/transcript_stream',
            self.transcript_callback,
            10)

        self.subscription  # prevent unused variable warning

    def transcript_callback(self, msg: AudioTranscript):
        threshold = self.get_parameter('threshold').value

        filtered_transcript = []
        for seg_i in range(len(msg.seg_start_words_id)):
            if len(filtered_transcript) > 0:
                filtered_transcript.append("\n")

            # Add timestamp and duration of segment
            filtered_transcript.append("[")
            filtered_transcript.append(time_to_string(msg.seg_start_time[seg_i]))
            filtered_transcript.append(" (")
            filtered_transcript.append(f"{msg.seg_duration_ms[seg_i]}")
            filtered_transcript.append(" ms)]:  ")

            seg_begin = msg.seg_start_words_id[seg_i]
            if seg_i == len(msg.seg_start_words_id) - 1:
                seg_end = len(msg.words)
            else:
                seg_end = msg.seg_start_words_id[seg_i+1]

            # Add words to the transcript
            for i in range(seg_begin, seg_end):
                likelyhood = msg.probs[i] * msg.occ[i]
                if i < msg.active_index:
                    if likelyhood >= threshold:
                        filtered_transcript.append(msg.words[i])
                else:
                    filtered_transcript.append(
                                    color_gradient(msg.words[i], likelyhood, 0, threshold))


        transcript_str = ''.join(filtered_transcript)
        os.system('clear')
        print(transcript_str)

def main(args=None):
    rclpy.init(args=args)

    # Create the node and spin it
    transcript_subscriber = TranscriptSubscriber()

    rclpy.spin(transcript_subscriber)

    # Destroy the node explicitly
    transcript_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
