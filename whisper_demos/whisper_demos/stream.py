import rclpy
from rclpy.node import Node
from whisper_idl.msg import AudioTranscript
import os
from termcolor import colored

class TranscriptSubscriber(Node):

    def __init__(self):
        super().__init__('transcript_subscriber')

        # Declare the threshold parameter with a default value of 1
        self.declare_parameter('threshold', 2.0)
        
        # Subscribe to the audio transcript topic
        self.subscription = self.create_subscription(
            AudioTranscript,
            '/whisper/transcript_stream',
            self.transcript_callback,
            10)

        self.subscription  # prevent unused variable warning
    
    def transcript_callback(self, msg: AudioTranscript):
        # Fetch the threshold value from the parameter
        threshold = self.get_parameter('threshold').value

        # Clear the screen before printing the transcript
        os.system('clear')

        # Process the words and filter them based on prob[i] * occ[i] >= threshold
        filtered_transcript = []
        for i in range(len(msg.words)):
            confidence_score = msg.probs[i] * msg.occ[i]
            if i < msg.active_index:
                if confidence_score >= threshold:
                    filtered_transcript.append(colored(msg.words[i], "white"))
            else:
                color = self.get_confidence_color(confidence_score, threshold)
                filtered_transcript.append(colored(msg.words[i], color))
            # if confidence_score >= threshold:
            #     # Color the text based on confidence score
            #     color = self.get_confidence_color(confidence_score, threshold)
            #     filtered_transcript.append(colored(msg.words[i], color))
        
        # Join the filtered words and print them
        transcript_str = ''.join(filtered_transcript)
        print(transcript_str)

    def get_confidence_color(self, score, threshold):
        # A score close to the threshold should be green, and closer to zero should be red
        if score >= threshold:
            return 'green'
        elif score >= threshold * 0.75:
            return 'yellow'
        elif score >= threshold * 0.5:
            return 'yellow'  # Adjusting color gradient
        else:
            return 'red'

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
