#include "transcript_manager/transcript_manager_node.hpp"

namespace whisper {
TranscriptManagerNode::TranscriptManagerNode(const rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_(node_ptr) {

  // Subscribe to incoming token data
  auto cb_group = node_ptr_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = cb_group;
  tokens_sub_ = node_ptr_->create_subscription<WhisperTokens>(
    "tokens", rclcpp::SensorDataQoS(), 
    std::bind(&TranscriptManagerNode::on_whisper_tokens_, this, std::placeholders::_1), options);

  // Action Server
  inference_action_server_ = rclcpp_action::create_server<Inference>(
    node_ptr_, "inference",
    std::bind(&TranscriptManagerNode::on_inference_, this, 
                                          std::placeholders::_1, std::placeholders::_2),
    std::bind(&TranscriptManagerNode::on_cancel_inference_, this, std::placeholders::_1),
    std::bind(&TranscriptManagerNode::on_inference_accepted_, this, std::placeholders::_1));

  // Data Initialization
  incoming_queue_ = std::make_unique<ThreadSafeRing<std::vector<Word>>>(10);
  transcript_ = std::make_unique<Transcript>(4, node_ptr);

  // Outgoing data pub
  transcript_pub_ = node_ptr_->create_publisher<AudioTranscript>("transcript_stream", 10);

  clear_queue_timer_ = node_ptr_->create_wall_timer(std::chrono::milliseconds(1000), 
                std::bind(&TranscriptManagerNode::clear_queue_callback_, this));
}
void TranscriptManagerNode::clear_queue_callback_() {
  clear_queue_();
}

void TranscriptManagerNode::on_whisper_tokens_(const WhisperTokens::SharedPtr msg) {
  // print_timestamp_(ros_msg_to_chrono(msg->stamp));
  // print_msg_(msg);
  const auto &words = deserialize_msg_(msg);
  // print_new_words_(words);

  incoming_queue_->enqueue(words);
  if (incoming_queue_->almost_full()) {
    auto& clk = *node_ptr_->get_clock();
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000,
                             "Transcripiton buffer full.  Dropping data.");
  }
}

rclcpp_action::GoalResponse TranscriptManagerNode::on_inference_(
                              const rclcpp_action::GoalUUID & /*uuid*/,
                             std::shared_ptr<const Inference::Goal> /*goal*/) {
  RCLCPP_INFO(node_ptr_->get_logger(), "Received inference request.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TranscriptManagerNode::on_cancel_inference_(
            const std::shared_ptr<GoalHandleInference> /*goal_handle*/) {
  RCLCPP_INFO(node_ptr_->get_logger(), "Cancelling inference...");
  return rclcpp_action::CancelResponse::ACCEPT;
}


void TranscriptManagerNode::on_inference_accepted_(
                          const std::shared_ptr<GoalHandleInference> goal_handle) {
  RCLCPP_INFO(node_ptr_->get_logger(), "Starting inference...");
  auto feedback = std::make_shared<Inference::Feedback>();
  auto result = std::make_shared<Inference::Result>();
  inference_start_time_ = node_ptr_->now();
  auto batch_idx = 0;
  while (rclcpp::ok()) {
    if (node_ptr_->now() - inference_start_time_ > goal_handle->get_goal()->max_duration) {
      result->info = "Inference timed out.";
      RCLCPP_INFO(node_ptr_->get_logger(), result->info.c_str());
      goal_handle->succeed(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      result->info = "Inference cancelled.";
      RCLCPP_INFO(node_ptr_->get_logger(), result->info.c_str());
      goal_handle->canceled(result);
      return;
    }

    // Wait for other thread
    while ( incoming_queue_->empty() ) {
      rclcpp::sleep_for(std::chrono::milliseconds(15));
    }

    // Clear queue
    std::string message;
    while ( !incoming_queue_->empty() ) {
      const auto words = incoming_queue_->dequeue();
      for (const auto &word : words) {
        if ( !word.is_segment() ) {
          message += word.get();
        }
      }
    }

    feedback->transcription = message;
    feedback->batch_idx = batch_idx;
    goal_handle->publish_feedback(feedback);
    result->transcriptions.push_back(feedback->transcription);
    RCLCPP_INFO(node_ptr_->get_logger(), "Batch %d", batch_idx);
    ++batch_idx;
  }

  if (rclcpp::ok()) {
    result->info = "Inference succeeded.";
    RCLCPP_INFO(node_ptr_->get_logger(), result->info.c_str());
    goal_handle->succeed(result);
  }
}


void TranscriptManagerNode::clear_queue_() {
  bool one_merged = false;
  while ( !incoming_queue_->empty() ) {
    one_merged = true;
    const auto words_and_segments = incoming_queue_->dequeue();
    // RCLCPP_INFO(node_ptr_->get_logger(), "Merging %ld words", words_and_segments.first.size());
    transcript_->merge_one_(words_and_segments);
  }

  if ( one_merged ) {
    // Publish new transcript
    auto message = AudioTranscript();
    serialize_transcript_(message);
    transcript_pub_->publish(message);

    {
      // Debug Print
      // const auto print_str = transcript_.get_print_str();
      // RCLCPP_INFO(node_ptr_->get_logger(), "Current Transcript:   \n%s", print_str.c_str());
    }
  }
}

void TranscriptManagerNode::serialize_transcript_(AudioTranscript &msg) {
  int words_skipped = 0; // Skip adding segments into the serialized word array
  for (auto it = transcript_->begin(); it != transcript_->end(); ++it) {
    const auto & word = *it;
    if (word.is_segment()) {
      const auto &segment_data = word.get_segment_data();
      msg.seg_start_words_id.push_back(msg.words.size());
      msg.seg_start_time.push_back(chrono_to_ros_msg(segment_data->get_start()));
      msg.seg_duration_ms.push_back(segment_data->get_duration().count());
      words_skipped++;
    } else {
      msg.words.push_back(word.get());
      msg.probs.push_back(word.get_prob());
      msg.occ.push_back(word.get_occurrences());
    }
  }
  msg.active_index = transcript_->get_stale_word_id() - words_skipped;
}

void TranscriptManagerNode::print_msg_(const WhisperTokens::SharedPtr &msg) {
  std::string print_str;

  print_str += "Inference Duration:  ";
  print_str += std::to_string(msg->inference_duration);
  print_str += "\n";

  print_str += "Segment starts:  ";
  for (size_t i=0; i<msg->segment_start_token_idxs.size(); ++i) {
    print_str += std::to_string(msg->segment_start_token_idxs[i]) + ", ";
  }
  print_str += "\n";


  bool first_token = true;
  int segment_ptr = 0;
  for (size_t i=0; i<msg->token_texts.size(); ++i) {

    // If token is start of new segment
    if (static_cast<size_t>(segment_ptr) < msg->segment_start_token_idxs.size() && 
              i == static_cast<size_t>(msg->segment_start_token_idxs[segment_ptr])) {

      // First segment
      if (segment_ptr != 0) {
        print_str += "\n"; 
      }

      // Last segment
      int segment_tokens;
      if (static_cast<size_t>(segment_ptr + 1) == msg->segment_start_token_idxs.size()) {
        segment_tokens = msg->token_texts.size() - msg->segment_start_token_idxs[segment_ptr];
      } else {
        segment_tokens = msg->segment_start_token_idxs[segment_ptr + 1] - 
                                      msg->segment_start_token_idxs[segment_ptr];
      }

      // Segment data
      print_str += "Segment Tokens: ";
      print_str += std::to_string(segment_tokens);
      print_str += "  Duration: ";
      print_str += std::to_string(msg->end_times[segment_ptr] - msg->start_times[segment_ptr]);
      print_str += "  Data: ";

      first_token = true; // Dont print "|"
      ++segment_ptr;
    }

    if (!first_token) {
      print_str += "|";
    }

    print_str += msg->token_texts[i];
    first_token = false;
  }

  print_str += "\n";
  RCLCPP_INFO(node_ptr_->get_logger(), "%s", print_str.c_str());
}

void TranscriptManagerNode::print_new_words_(const std::vector<Word> &new_words) {
  std::string print_str;
  bool first_print = true;
  for (size_t i = 0; i < new_words.size();  ++i) {
    const auto &word = new_words[i];
    if (word.is_segment()) {
      const auto seg = word.get_segment_data();
      print_str += "\n";
      print_str += seg->as_str();
      first_print = true;
      continue;
    }
    if (!first_print) {
      print_str += "||";
    }
    print_str += word.get();
    first_print = false;
  }
  print_str += "\n";
  RCLCPP_INFO(node_ptr_->get_logger(), "%s", print_str.c_str());
}


void TranscriptManagerNode::print_timestamp_(std::chrono::system_clock::time_point timestamp) {
  std::time_t time_t_val = std::chrono::system_clock::to_time_t(timestamp);
  auto duration_since_epoch = timestamp.time_since_epoch();
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration_since_epoch);
  auto milliseconds = 
          std::chrono::duration_cast<std::chrono::milliseconds>(duration_since_epoch - seconds);
  std::tm* tm = std::localtime(&time_t_val);
  RCLCPP_INFO(node_ptr_->get_logger(), "RECIEVED:  %04d-%02d-%02d %02d:%02d:%02d.%03d\n",
         tm->tm_year + 1900,    // Year
         tm->tm_mon + 1,        // Month (0-based in tm struct)
         tm->tm_mday,           // Day
         tm->tm_hour,           // Hour
         tm->tm_min,            // Minute
         tm->tm_sec,            // Second
         static_cast<int>(milliseconds.count()));  // Milliseconds
}


std::vector<Word> 
    TranscriptManagerNode::deserialize_msg_(const WhisperTokens::SharedPtr &msg) {
  std::vector<Word> words;
  std::vector<SingleToken> word_wip;

  auto audio_start = ros_msg_to_chrono(msg->stamp);
  
  size_t segment_ptr = 0;
  for (size_t i=0; i<msg->token_texts.size(); ++i) {
    // RCLCPP_INFO(node_ptr_->get_logger(), "i: %ld.  Token:  '%s'", i, msg->token_texts[i].c_str());

    // 
    // Deserialize Segment Data
    // 
    if (segment_ptr < msg->segment_start_token_idxs.size() &&
            i == static_cast<size_t>(msg->segment_start_token_idxs[segment_ptr])) {
      // Complete previous word before starting new segment
      if (!word_wip.empty()) {
        words.push_back({word_wip});
        word_wip.clear();
        // RCLCPP_INFO(node_ptr_->get_logger(), " NEW SEGMENT     --- Added a word");
      }

      // Get the segment end token
      size_t end_token_id;
      if (segment_ptr == msg->segment_start_token_idxs.size()-1) {
        // last segment
        end_token_id = msg->token_texts.size()-1;
      } else {
        end_token_id = static_cast<size_t>(msg->segment_start_token_idxs[segment_ptr+1] - 1);
      }
      SingleToken end_token(msg->token_texts[end_token_id], msg->token_probs[end_token_id]);


      // Create segment with:  {End token, Duration, Start timestamp}
      std::chrono::milliseconds start_ms(msg->start_times[segment_ptr]*whisper_ts_to_ms_ratio);
      std::chrono::milliseconds end_ms(msg->end_times[segment_ptr]*whisper_ts_to_ms_ratio);
      SegmentMetaData segment(end_token, end_ms - start_ms, audio_start + start_ms);
      words.push_back({segment});
      ++segment_ptr;
    }

    // 
    // Deserialize Token Data
    // 
    // Decide if we should start a new word
    if (!word_wip.empty() && !msg->token_texts[i].empty()) {
      // RCLCPP_INFO(node_ptr_->get_logger(), "   Checking for space");
      if (std::isspace(msg->token_texts[i][0])) {
        words.push_back({word_wip});
        word_wip.clear();
        // RCLCPP_INFO(node_ptr_->get_logger(), "     --- Added new word");
      }
    }

    if (is_special_token(msg->token_texts, i)) {
      // Skip whisper special tokens (e.g. [_TT_150_])
    }
    else if (my_ispunct(msg->token_texts, i)) {
      // Push back last word
      words.push_back({word_wip});
      word_wip.clear();
      // RCLCPP_INFO(node_ptr_->get_logger(), "Punct     --- Added new word");
      // Add punctuation as its own word
      words.push_back({SingleToken(msg->token_texts[i], msg->token_probs[i]), true});
    }
    else if (auto [join, num_tokens] = join_tokens(msg->token_texts, i); join) {
      std::string combined_text = combine_text(msg->token_texts, i, num_tokens);
      float combined_prob = combine_prob(msg->token_probs, i, num_tokens);
      
      // RCLCPP_INFO(node_ptr_->get_logger(), "Combining next %d tokens-> '%s'", 
      //                                                 num_tokens, combined_text.c_str());
      word_wip.push_back(SingleToken(combined_text, combined_prob));
      i += num_tokens - 1; // Skip next tokens in loop
    }
    else {
      word_wip.push_back(SingleToken(msg->token_texts[i], msg->token_probs[i]));
    }
  }

  // Final word
  if (!word_wip.empty()) {
    words.push_back({word_wip});
  }

  return words;
}


// 
// Helper Functions
//
bool TranscriptManagerNode::is_special_token(const std::vector<std::string> &tokens, 
                                                                          const int idx) {
  const std::vector<std::string> special_token_start_strs = {
      "[_BEG_]", "[_TT_", " [_BEG_]", " [_TT_"
  };
  for (const auto &start : special_token_start_strs) { 
    if ( tokens[idx].size() >= start.size() && tokens[idx].substr(0, start.size()) == start ) {
      return true;
    }
  }
  return false;
}

bool TranscriptManagerNode::my_ispunct(const std::vector<std::string> &tokens, const int idx) {
  // The reason for not using std::punct(..) on the first character is that "'t" would 
  //    be considered a punctuation.
  // As well as some brackets which should be combined into words.
  // Otherwise it is missing "..." which we can consider as punctuation
  const std::vector<std::string> punctuations = {",", ".", "?", "!", ":", ";", "...", "+", "-"};
  return std::find(punctuations.begin(), punctuations.end(), tokens[idx]) != punctuations.end();
}

bool TranscriptManagerNode::contains_char(const std::string &str, const char target) {
  for (const auto &c : str) {
    if ( target == c ) {
      return true;
    }
  }
  return false;
}

std::pair<bool, int> TranscriptManagerNode::join_tokens(const std::vector<std::string> &tokens, 
                                                                                const int idx) {
  // Check if, starting from the idx, the tokens are a bracket which can be combined or removed
  //   Return:
  //      bool -- Tokens start with bracket (can be combined)
  //      int  -- Number of tokens to combine
  const std::vector<std::pair<char, char>> combine_brackets = {
      {'[', ']'},
      {'{', '}'},
      {'(', ')'}
  };

  // Try to combine tokens
  for (auto &[start, end] : combine_brackets) { 
    if ( contains_char(tokens[idx], start) ) {
      int end_idx = idx + 1;
      while (end_idx < static_cast<int>(tokens.size()) && (end_idx-idx) 
                                            <= max_number_tokens_to_combine) {
        if ( contains_char(tokens[end_idx], end) ) {
          return {true, end_idx - idx + 1};
        }
        end_idx++;
      }
      // We found the start but not the end, "combine" current token with itself
      return {true, 1};
    }
  }
  return {false, 0};
}

std::string TranscriptManagerNode::combine_text(const std::vector<std::string> &tokens, 
                                                      const int idx, const int num) {
  std::string ret = "";
  for (auto i = idx; i < idx + num; ++i) {
    ret += tokens[i];
  }
  return ret;
}

float TranscriptManagerNode::combine_prob(const std::vector<float> &probs, 
                                                      const int idx, const int num) {
  float ret = 0.;
  for (auto i = idx; i < idx + num; ++i) {
    ret += probs[i];
  }
  return ret / num;
}

} // end of namespace whisper
