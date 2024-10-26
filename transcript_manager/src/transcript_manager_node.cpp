#include "transcript_manager/transcript_manager_node.hpp"

namespace whisper {
TranscriptManagerNode::TranscriptManagerNode(const rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_(node_ptr) {

  // Subscribe to incoming token data
  auto cb_group = node_ptr_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = cb_group;
  tokens_sub_ = node_ptr_->create_subscription<WhisperTokens>(
    "tokens", 10, 
    std::bind(&TranscriptManagerNode::on_whisper_tokens_, this, std::placeholders::_1), options);

  // Action Server
  inference_action_server_ = rclcpp_action::create_server<Inference>(
    node_ptr_, "inference",
    std::bind(&TranscriptManagerNode::on_inference_, this, 
                                          std::placeholders::_1, std::placeholders::_2),
    std::bind(&TranscriptManagerNode::on_cancel_inference_, this, std::placeholders::_1),
    std::bind(&TranscriptManagerNode::on_inference_accepted_, this, std::placeholders::_1));

  // Data Initialization
  incoming_queue_ = 
    std::make_unique<ThreadSafeRing<std::pair<std::vector<Word>, std::vector<SegmentMetaData>>>>(
                                                      10);
}

void TranscriptManagerNode::on_whisper_tokens_(const WhisperTokens::SharedPtr msg) {
  // auto last_msg = 
  //     std::accumulate(msg->token_texts.begin(), msg->token_texts.end(), std::string());
  // RCLCPP_INFO(node_ptr_->get_logger(), "Transcript: %s", last_msg.c_str());

  // print_msg_(msg);
  auto [words, segments] = deserialize_msg_(msg);
  // print_new_words_(words, segments);

  incoming_queue_->enqueue({words, segments});
  if (incoming_queue_->almost_full()) {
    auto& clk = *node_ptr_->get_clock();
    RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), clk, 5000,
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
      auto [words, segments] = incoming_queue_->dequeue();
      for (auto &word : words) {
        message += word.get();
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

void TranscriptManagerNode::print_new_words_(const std::vector<Word> &new_words,
                                            const std::vector<SegmentMetaData> &new_segments) {
  std::string print_str;
  for (const auto &seg : new_segments) {
    print_str += "[" + std::to_string(seg.start_time_) + 
                " - " + std::to_string(seg.end_time_) + "]:  ";
    bool first_print = true;
    for (int i = seg.word_array_start_idx_; i < (seg.word_array_start_idx_ + seg.len_); ++i) {
      // RCLCPP_INFO(node_ptr_->get_logger(), "i: %d.  word :  '%s'", i, new_words[i].get().c_str());
      if (new_words[i].is_segment()) {
        continue;
      }
      if (!first_print) {
        print_str += "||";
      }
      print_str += new_words[i].get();
      first_print = false;
    }
    print_str += "\n";
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "%s", print_str.c_str());
}



std::pair<std::vector<Word>, std::vector<SegmentMetaData>> 
                  TranscriptManagerNode::deserialize_msg_(const WhisperTokens::SharedPtr &msg) {
  std::vector<Word> words;
  std::vector<SegmentMetaData> segments;

  SegmentMetaData segment_wip;
  std::vector<SingleToken> word_wip;

  int segment_ptr = 0;
  bool segment_started = false;
  for (size_t i=0; i<msg->token_texts.size(); ++i) {

    // RCLCPP_INFO(node_ptr_->get_logger(), "i: %ld.  Token:  '%s'", i, msg->token_texts[i].c_str());
    // 
    // Deserialize Segment Data
    // 
    if (static_cast<size_t>(segment_ptr) < msg->segment_start_token_idxs.size() &&
            i == static_cast<size_t>(msg->segment_start_token_idxs[segment_ptr])) {
      // Complete previous word before starting new segment
      if (!word_wip.empty()) {
        words.push_back({word_wip});
        word_wip.clear();
        // RCLCPP_INFO(node_ptr_->get_logger(), " NEW SEGMENT     --- Added a word");
      }

      // If we are starting on the second segment, fill in information from the previous
      if (segment_ptr >= 1 && segment_started) {
        segment_wip.len_ = static_cast<int>(words.size()) - segment_wip.word_array_start_idx_;
        // Save the previous segment, reset the wip
        // RCLCPP_INFO(node_ptr_->get_logger(), "Push back Segment with end token:  %s", 
        //                                                   segment_wip.get_data().c_str());
        segments.push_back(segment_wip);
        segment_wip = SegmentMetaData();
        segment_started = false;
      }
      // Fill in information about the new segment
      // Get the current segment end token
      int segment_tokens;
      if (static_cast<size_t>(segment_ptr + 1) == msg->segment_start_token_idxs.size()) {
        // Last segment
        segment_tokens = msg->token_texts.size() - msg->segment_start_token_idxs[segment_ptr];
      } else {
        // Compute offset to next segment
        segment_tokens = msg->segment_start_token_idxs[segment_ptr + 1] - 
                                      msg->segment_start_token_idxs[segment_ptr];
      }

      if (segment_tokens > 0) {
        segment_wip.word_array_start_idx_ = static_cast<int>(words.size());
        segment_wip.end_token_ = SingleToken(msg->token_texts[i + segment_tokens - 1], 
                                             msg->token_probs[i + segment_tokens - 1]);
        segment_wip.start_time_ = msg->start_times[segment_ptr];
        segment_wip.end_time_ = msg->end_times[segment_ptr];
        // RCLCPP_INFO(node_ptr_->get_logger(), "Segment end token:  %s", 
        //                                                 segment_wip.get_data().c_str());
        // Push indicator of new segment
        words.push_back(Word());
        segment_started = true;
      }

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
      // Do not add whisper special tokens to transcript
      // Store as segment data
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

  // Final segment
  if (segment_started) {
    // Dont subtract one since no new segment began
    segment_wip.len_ = static_cast<int>(words.size()) - segment_wip.word_array_start_idx_;
    // Save the previous segment, reset the wip
    // RCLCPP_INFO(node_ptr_->get_logger(), "Push back Segment with end token:  %s", 
    //                                                               segment_wip.get_data().c_str());
    segments.push_back(segment_wip);
  }

  return {words, segments};
}


} // end of namespace whisper
