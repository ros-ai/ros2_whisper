builtin_interfaces/Time stamp

# Token data
int32[] token_ids
string[] token_texts
float32[] token_probs

# Segment data
int32[] segment_start_token_idxs
int64[] start_times
int64[] end_times

# Runtime data
int64 inference_duration

# Audio Source (TODO)
