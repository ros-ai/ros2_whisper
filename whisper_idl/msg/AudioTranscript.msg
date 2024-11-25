# File:  AudioTranscript.msg

# Text data
string[] words                             # The word from speech-to-text
float32[] probs                            # Confidence value 
int32[] occ                                # Word occurances 

# Segment Data
int32[] seg_start_words_id                 # Location in the words array where the segment starts
builtin_interfaces/Time[] seg_start_time   # Start time of the segment
int32[] seg_duration_ms                    # Segment duration in ms 

# Meta
int32 active_index                         # All words past this index in the transcript may change