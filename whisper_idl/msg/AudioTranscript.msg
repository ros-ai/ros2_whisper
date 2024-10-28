# File:  AudioTranscript.msg

string[] words                        # The word from speech-to-text
float32[] probs    		              # Confidence value 
int32[] occ    		                  # Word occurances 
int32 active_index					  # All words past this index in the transcript may change
builtin_interfaces/Time[] timestamps  # TODO:  Timestamp value for each word