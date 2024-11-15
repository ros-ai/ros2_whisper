#include "transcript_manager/transcript_manager.hpp"

namespace whisper {
TranscriptManager::TranscriptManager(const rclcpp::NodeOptions& options)
    : Node("transcript_manager", options) {

  // Declare merge algorithm parameter
  declare_parameter("allowed_lcs_gaps", 4);

  // Subscribe to incoming token data
  auto cb_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group;
  tokens_sub_ = create_subscription<WhisperTokens>("tokens", 10,
    std::bind(&TranscriptManager::on_whisper_tokens_, this, std::placeholders::_1), sub_options);

  // Action Server
  inference_action_server_ = rclcpp_action::create_server<Inference>(
    this, "inference",
    std::bind(&TranscriptManager::on_inference_, this, 
                                          std::placeholders::_1, std::placeholders::_2),
    std::bind(&TranscriptManager::on_cancel_inference_, this, std::placeholders::_1),
    std::bind(&TranscriptManager::on_inference_accepted_, this, std::placeholders::_1));

  // Data Initialization
  incoming_queue_ = std::make_unique<ThreadSafeRing<std::vector<Segment>>>(10);
  int allowed_lcs_gaps = get_parameter("allowed_lcs_gaps").as_int();
  // How to get a node pointer from a component:
  // https://robotics.stackexchange.com/questions/102145/how-to-initialize-image-transport-using-rclcpp
  rclcpp::Node::SharedPtr node_handle_ = std::shared_ptr<TranscriptManager>(this, [](auto *) {});
  transcript_ = std::make_unique<Transcript>(allowed_lcs_gaps, node_handle_);

  // Outgoing data pub
  transcript_pub_ = create_publisher<AudioTranscript>("transcript_stream", 10);
  clear_queue_timer_ = create_wall_timer(std::chrono::milliseconds(1000), 
                std::bind(&TranscriptManager::clear_queue_callback_, this), cb_group);
}

void TranscriptManager::clear_queue_callback_() {
  clear_queue_();
}

void TranscriptManager::on_whisper_tokens_(const WhisperTokens::SharedPtr msg) {
  // print_msg_(msg);
  const auto &words_and_segments = deserialize_msg_(msg);
  // print_new_words_(words_and_segments);

  incoming_queue_->enqueue(words_and_segments);
  if (incoming_queue_->almost_full()) {
    auto& clk = *get_clock();
    RCLCPP_WARN_THROTTLE(get_logger(), clk, 5000,
                             "Transcripiton buffer full.  Dropping data.");
  }
}

rclcpp_action::GoalResponse TranscriptManager::on_inference_(
                              const rclcpp_action::GoalUUID & /*uuid*/,
                             std::shared_ptr<const Inference::Goal> /*goal*/) {
  RCLCPP_INFO(get_logger(), "Received inference request.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TranscriptManager::on_cancel_inference_(
            const std::shared_ptr<GoalHandleInference> /*goal_handle*/) {
  RCLCPP_INFO(get_logger(), "Cancelling inference...");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TranscriptManager::on_inference_accepted_(
                          const std::shared_ptr<GoalHandleInference> goal_handle) {
  RCLCPP_INFO(get_logger(), "Starting inference...");
  auto feedback = std::make_shared<Inference::Feedback>();
  auto result = std::make_shared<Inference::Result>();
  inference_start_time_ = now();
  auto batch_idx = 0;
  transcript_->clear();
  size_t last_stale_seg = transcript_->get_stale_segment();

  // Helper lambda function
  auto fill_result = [this, &result](const std::string& info_msg, const size_t last_stale_seg) {
    result->info = info_msg;
    RCLCPP_INFO(get_logger(), result->info.c_str());
    for (auto it = transcript_->segments_begin() + last_stale_seg;
                              it != transcript_->segments_end(); ++it) {
      result->transcriptions.push_back(it->get_words());
    }
  };

  while (rclcpp::ok()) {
    if ( now() - inference_start_time_ > goal_handle->get_goal()->max_duration ) {
      fill_result(std::string("Inference timed out."), last_stale_seg);
      goal_handle->succeed(result);
      return;
    }

    if ( goal_handle->is_canceling() ) {
      fill_result(std::string("Inference cancelled."), last_stale_seg);
      goal_handle->canceled(result);
      return;
    }

    // Check for changes on the stale segment marker
    size_t new_stale_segment = transcript_->get_stale_segment();
    if ( last_stale_seg != new_stale_segment ) {
      // Add fialized transcription to result
      for (auto it = transcript_->segments_begin() + last_stale_seg;
                                it < transcript_->segments_begin() + new_stale_segment; ++it) {
        result->transcriptions.push_back(it->get_words());
      }
      last_stale_seg = new_stale_segment;
    }

    // Give feedback of active transcription
    std::string active_transcript;
    for (auto it = transcript_->segments_begin() + last_stale_seg;
                              it != transcript_->segments_end(); ++it) {
      active_transcript += it->get_words();
    }

    feedback->transcription = active_transcript;
    feedback->batch_idx = batch_idx;
    goal_handle->publish_feedback(feedback);
    ++batch_idx;

    rclcpp::sleep_for(std::chrono::milliseconds(250));
  }

  if ( rclcpp::ok() ) {
    fill_result(std::string("Inference succeeded."), last_stale_seg);
    goal_handle->succeed(result);
  }
}

bool TranscriptManager::clear_queue_() {
  bool one_merged = false;
  while ( !incoming_queue_->empty() ) {
    one_merged = true;
    const auto words_and_segments = incoming_queue_->dequeue();
    transcript_->merge_one_(words_and_segments);
  }

  if ( one_merged ) {
    // Publish new transcript
    auto message = AudioTranscript();
    serialize_transcript_(message);
    transcript_pub_->publish(message);

    // const auto print_str = transcript_->get_print_str();
    // RCLCPP_INFO(get_logger(), "Current Transcript:   \n%s\n", print_str.c_str());
  }
  return one_merged;
}

void TranscriptManager::serialize_transcript_(AudioTranscript &msg) {
  size_t stale_counter = 0;
  size_t segment_count = 0;
  size_t stale_segment_id = transcript_->get_stale_seg_id();
  for (auto it = transcript_->segments_begin(); it != transcript_->segments_end(); ++it) {
    auto segment = *it;
    msg.seg_start_words_id.push_back(msg.words.size());
    msg.seg_start_time.push_back(chrono_to_ros_msg(segment.get_start()));
    msg.seg_duration_ms.push_back(segment.get_duration().count());
    for (const auto& word : segment.words_) {
      msg.words.push_back(word.get());
      msg.probs.push_back(word.get_prob());
      msg.occ.push_back(word.get_occurrences());
    }
    if (segment_count < stale_segment_id) {
      stale_counter += segment.words_.size();
    }
    segment_count++;
  }
  msg.active_index = stale_counter;
}

std::vector<Segment> 
    TranscriptManager::deserialize_msg_(const WhisperTokens::SharedPtr &msg) {
  std::vector<SingleToken> word_wip;
  Segment segment_wip;
  std::vector<Segment> segments;

  auto audio_start = ros_msg_to_chrono(msg->stamp);
  
  size_t segment_ptr = 0;
  for (size_t i=0; i<msg->token_texts.size(); ++i) {
    // 
    // Deserialize Segment Data
    // 
    if ( segment_ptr < msg->segment_start_token_idxs.size() &&
            i == static_cast<size_t>(msg->segment_start_token_idxs[segment_ptr]) ) {
      // Complete previous word before starting new segment
      if ( !word_wip.empty() ) {
        segment_wip.words_.push_back({word_wip});
        word_wip.clear();
      }

      // Add a non-empty, completed segment
      if ( segment_wip.words_.size() > 0 ) {
        segments.push_back(segment_wip);
      }

      // Set up for an new segment
      segment_wip.clear();

      // Get the segment end token
      size_t end_token_id;
      if ( segment_ptr == msg->segment_start_token_idxs.size()-1 ) {
        // last segment
        end_token_id = msg->token_texts.size()-1;
      } else {
        end_token_id = static_cast<size_t>(msg->segment_start_token_idxs[segment_ptr+1] - 1);
      }
      SingleToken end_token(msg->token_texts[end_token_id], msg->token_probs[end_token_id]);

      // Create segment with:  {End token, Duration, Start timestamp}
      std::chrono::milliseconds start_ms(msg->start_times[segment_ptr]*whisper_ts_to_ms_ratio);
      std::chrono::milliseconds end_ms(msg->end_times[segment_ptr]*whisper_ts_to_ms_ratio);
      segment_wip.data_.start_ = audio_start + start_ms;
      segment_wip.data_.duration_ = end_ms - start_ms;
      segment_wip.data_.end_token_ = end_token;
      ++segment_ptr;
    }

    // 
    // Deserialize Token Data
    // 
    // Decide if we should start a new word
    if ( !word_wip.empty() && !msg->token_texts[i].empty() ) {
      if ( std::isspace(msg->token_texts[i][0]) ) {
        segment_wip.words_.push_back({word_wip});
        word_wip.clear();
      }
    }

    if ( is_special_token(msg->token_texts, i) ) {
      // Skip whisper special tokens (e.g. [_TT_150_])
    }
    else if ( my_ispunct(msg->token_texts, i) ) {
      // Push back last word
      segment_wip.words_.push_back({word_wip});
      word_wip.clear();
      // Add punctuation as its own word
      segment_wip.words_.push_back({SingleToken(msg->token_texts[i], msg->token_probs[i]), true});
    }
    else if ( auto [join, num_tokens] = join_tokens(msg->token_texts, i); join ) {
      std::string combined_text = combine_text(msg->token_texts, i, num_tokens);
      float combined_prob = combine_prob(msg->token_probs, i, num_tokens);
      word_wip.push_back(SingleToken(combined_text, combined_prob));
      i += num_tokens - 1; // Skip next tokens in loop
    }
    else {
      word_wip.push_back(SingleToken(msg->token_texts[i], msg->token_probs[i]));
    }
  }

  // Final word
  if ( !word_wip.empty() ) {
    segment_wip.words_.push_back({word_wip});
  }

  // Finish by adding completed segment
  if ( !segment_wip.words_.empty() ) {
    segments.push_back(segment_wip);
  }

  return segments;
}

// 
// Helper Functions
//
bool TranscriptManager::is_special_token(const std::vector<std::string> &tokens, 
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

bool TranscriptManager::my_ispunct(const std::vector<std::string> &tokens, const int idx) {
  // The reason for not using std::punct(..) on the first character is that "'t" would 
  //    be considered a punctuation.
  // As well as some brackets which should be combined into words.
  // Otherwise it is missing "..." which we can consider as punctuation
  const std::vector<std::string> punctuations = {",", ".", "?", "!", ":", ";", "...", "+", "-"};
  return std::find(punctuations.begin(), punctuations.end(), tokens[idx]) != punctuations.end();
}

bool TranscriptManager::contains_char(const std::string &str, const char target) {
  for (const auto &c : str) {
    if ( target == c ) {
      return true;
    }
  }
  return false;
}

std::pair<bool, int> TranscriptManager::join_tokens(const std::vector<std::string> &tokens, 
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

std::string TranscriptManager::combine_text(const std::vector<std::string> &tokens, 
                                                      const int idx, const int num) {
  std::string ret = "";
  for (auto i = idx; i < idx + num; ++i) {
    ret += tokens[i];
  }
  return ret;
}

float TranscriptManager::combine_prob(const std::vector<float> &probs, 
                                                      const int idx, const int num) {
  float ret = 0.;
  for (auto i = idx; i < idx + num; ++i) {
    ret += probs[i];
  }
  return ret / num;
}



// 
// Print Functions
//

void TranscriptManager::print_msg_(const WhisperTokens::SharedPtr &msg) {
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
  RCLCPP_INFO(get_logger(), "%s", print_str.c_str());
}


void TranscriptManager::print_new_words_(const std::vector<Segment> &new_words) {
  std::string print_str;
  for (const auto& seg : new_words) {
    print_str += seg.as_str() + "\n";
  }
  RCLCPP_INFO(get_logger(), "%s", print_str.c_str());
}
} // end of namespace whisper


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(whisper::TranscriptManager)