#include "transcript_manager/transcript_manager_node.hpp"

namespace whisper {
TranscriptManagerNode::TranscriptManagerNode(const rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_(node_ptr), allowed_gaps(5) {

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
  incoming_queue_ = std::make_unique<ThreadSafeRing<WordsAndSegments>>(10);

  // Outgoing data pub
  transcript_pub_ = node_ptr_->create_publisher<AudioTranscript>("transcript_stream", 10);

  clear_queue_timer_ = node_ptr_->create_wall_timer(std::chrono::milliseconds(1000), 
                std::bind(&TranscriptManagerNode::clear_queue_callback_, this));
}
void TranscriptManagerNode::clear_queue_callback_() {
  // RCLCPP_INFO(node_ptr_->get_logger(), "Timer Callback.");
  clear_queue_();
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


void TranscriptManagerNode::merge_one_(const WordsAndSegments &new_words_and_segments) {
  auto stale_id = transcript_.get_stale_word_id();
  
  // 
  std::string tmp_print_str_1_;
  std::string tmp_print_str_2_;

  if (transcript_.empty()) {
    transcript_.push_back(new_words_and_segments);
    RCLCPP_DEBUG(node_ptr_->get_logger(), "First Words Added");
    return;
  }

  // Get comparable strings for fuzzy lcs matching
  auto [new_words, new_segments] = new_words_and_segments;
  auto old_words = transcript_.get_words_splice();
  std::vector<std::string> comp_words_old, comp_words_new;
  std::vector<int> skipped_ids_old, skipped_ids_new;
  size_t skipped_so_far = 0;
  for (const auto &word : old_words) {
    const auto &comp_word = word.get_comparable();
    if (comp_word.empty()) {
      skipped_so_far++;
    } else {
      comp_words_old.push_back(comp_word);  
      skipped_ids_old.push_back(static_cast<int>(skipped_so_far));
      tmp_print_str_1_ += "'" + comp_words_old[comp_words_old.size() - 1] + "', ";
    }
  }
  skipped_so_far = 0;
  for (size_t i = 0; i < new_words.size(); ++i) {
    const auto &comp_word = new_words[i].get_comparable();
    if (comp_word.empty()) {
      skipped_so_far++;
    } else {
      comp_words_new.push_back(comp_word);  
      skipped_ids_new.push_back(static_cast<int>(skipped_so_far));
      tmp_print_str_2_ += "'" + comp_words_new[comp_words_new.size() - 1] + "', ";
    }
  }
  RCLCPP_DEBUG(node_ptr_->get_logger(), " ");
  RCLCPP_DEBUG(node_ptr_->get_logger(), "Comp Against:  %s", tmp_print_str_1_.c_str());
  RCLCPP_DEBUG(node_ptr_->get_logger(), "   New Words:  %s", tmp_print_str_2_.c_str());

  // Longest Common Substring with Gaps.
  //   A:  Old words (already in Transcript), B:  New words recieved from live feed
  auto [indiciesA, indiciesB] = lcs_indicies_(comp_words_old, comp_words_new, allowed_gaps);
  if (indiciesA.empty()) {
    RCLCPP_DEBUG(node_ptr_->get_logger(), "  ---No overlap");
    transcript_.push_back(new_words_and_segments);
    return;
  }
  
  // Merge segments
  Transcript::Operations pending_ops;

  auto prevA = indiciesA[0], prevB = indiciesB[0];
  for(size_t i = 1; i <= indiciesA.size(); ++i) {
    // Include the offsets from skipped words
    auto prevA_id = prevA + skipped_ids_old[prevA];
    auto prevB_id = prevB + skipped_ids_new[prevB];
    RCLCPP_DEBUG(node_ptr_->get_logger(), "\tPrevA: %d,  PrevB:  %d:   %s (%d)", 
                                            prevA_id, prevB_id, 
                                            old_words[prevA_id].get().c_str(),
                                            old_words[prevA_id].get_occurrences());
    
    // Increment likely-hood of matched word in transcript
    pending_ops.push_back({Transcript::OperationType::INCREMENT, prevA_id, -1});

    // Current index "i" may not be valid
    int curA_id = prevA_id + 1, curB_id = prevB_id + 1;
    int nextA_id, nextB_id;
    if (i == indiciesA.size()) {
      // The following merge rules will run to the end of the new_words and old_words array.
      // Most commonly, new words that do not exist in the transcript are inserted at the end.
      nextA_id = old_words.size(), nextB_id = new_words.size();
    } else {
      nextA_id = indiciesA[i] + skipped_ids_old[indiciesA[i]];
      nextB_id = indiciesB[i] + skipped_ids_new[indiciesB[i]];
    }
    // RCLCPP_INFO(node_ptr_->get_logger(), "\tNextA: %d,  NextB:  %d", nextA_id, nextB_id);
    while (curA_id != nextA_id || curB_id != nextB_id) {
      // RCLCPP_INFO(node_ptr_->get_logger(), "\t\tCurA: %d,  CurB:  %d", curA_id, curB_id);
      // 
      // Custom Merge Rules
      // 
      // 0.  Skip start-of-segment markers
      if (curA_id != nextA_id && old_words[curA_id].is_segment()) {
        curA_id++;
        continue;
      }
      if (curB_id != nextB_id && new_words[curB_id].is_segment()) {
        curB_id++;
        continue;
      }
      // 1.  Encourage over-writing punctuation in the transcript (if the update is a word)
      if (curA_id != nextA_id && curB_id != nextB_id && 
            old_words[curA_id].is_punct() && ! new_words[curB_id].is_punct()) {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "\t\tWord conflict between "
                                    "transcript punctuation and update.  '%s' (%d - 1) --> '%s'",
                                            old_words[curA_id].get().c_str(),
                                            old_words[curA_id].get_occurrences(),
                                            new_words[curB_id].get().c_str());
        pending_ops.push_back({Transcript::OperationType::DECREMENT, curA_id, -1});
        pending_ops.push_back({Transcript::OperationType::CONFLICT, curA_id, curB_id});
        curA_id++; curB_id++;
      }
      // 2.  Conflict when there is a gap because of missmatched words in the LCS
      else if (curA_id != nextA_id && curB_id != nextB_id) {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "\t\tResolve Conflict Between '%s'(%d) and '%s'(%d)",
                                                  old_words[curA_id].get().c_str(), 
                                                  old_words[curA_id].get_occurrences(), 
                                                  new_words[curB_id].get().c_str(), 
                                                  new_words[curB_id].get_occurrences());
        // If we have a conflict, the word's likely-hood could be decreased.
        //    This causes some issues with words that sound the same (are constantly in conflict).
        // pending_ops.push_back({Transcript::OperationType::DECREMENT, curA_id});
        pending_ops.push_back({Transcript::OperationType::CONFLICT, curA_id, curB_id});
        curA_id++; curB_id++;
      }
      // 3.  Words appear in the audio steam (update) which are not part of the transcript
      else if (curB_id != nextB_id) {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "\t\tInserting word '%s' -- Between '%s' and '%s'",
                                            new_words[curB_id].get().c_str(), 
                                            old_words[curA_id-1].get().c_str(), 
                                            curA_id == static_cast<int>(old_words.size()) ? 
                                                        "END" : old_words[curA_id].get().c_str());

        pending_ops.push_back({Transcript::OperationType::INSERT, curA_id, curB_id});
        curB_id++;
      }
      // 4.  Words in the transcript are missing from the update
      else {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "\t\tDecreasing Likelihood of word:  '%s' (%d -> %d)", 
                                                old_words[curA_id].get().c_str(),
                                                old_words[curA_id].get_occurrences(),
                                                old_words[curA_id].get_occurrences() - 1);
        pending_ops.push_back({Transcript::OperationType::DECREMENT, curA_id, -1});
        curA_id++;
      }
    }
    // Prep for next loop
    prevA = indiciesA[i]; prevB = indiciesB[i];
  }

  transcript_.run(pending_ops, new_words_and_segments);
  transcript_.clear_mistakes(-1);

  auto stale_id_new = std::max(stale_id, stale_id + indiciesA[0] - indiciesB[0]);
  RCLCPP_DEBUG(node_ptr_->get_logger(), "Stale id update %d -> %d", stale_id, stale_id_new );
  transcript_.set_stale_word_id(stale_id_new);
}


/*
void TranscriptManagerNode::_backup_merge_one_(const WordsAndSegments &words_and_segments) {
  std::string tmp_print_str_1_;
  std::string tmp_print_str_2_;
  std::string tmp_print_str_3_;
  std::string tmp_print_str_4_;

  if (transcript_.empty()) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Adding First Words");
    transcript_.push_back(words_and_segments);
    RCLCPP_INFO(node_ptr_->get_logger(), "First Words Added");
    return;
  }
  // Get comparable strings for fuzzy lcs matching
  auto [words, segments] = words_and_segments;
  std::vector<std::string> comparable_words;
  std::vector<std::string> comparable_words_new;
  std::vector<int> skipped_ids_new;
  size_t skipped_so_far = 0;
  for (size_t i = 0; i < words.size(); ++i) {
    auto new_word_comp = words[i].get_comparable();
    if (new_word_comp.empty()) {
      skipped_so_far++;
    } else {
      comparable_words_new.push_back(new_word_comp);  
      skipped_ids_new.push_back(static_cast<int>(skipped_so_far));
      tmp_print_str_1_ += "'" + comparable_words_new[comparable_words_new.size()-1] + "', ";
    }
  }
  for (const auto &word : transcript_.get_words_splice()) {
    comparable_words.push_back(word.get_comparable());
    tmp_print_str_2_ += "'" + comparable_words[comparable_words.size()-1] + "', ";
  }
  RCLCPP_INFO(node_ptr_->get_logger(), " ");
  RCLCPP_INFO(node_ptr_->get_logger(), "Comp Against:  %s", tmp_print_str_2_.c_str());
  RCLCPP_INFO(node_ptr_->get_logger(), "   New Words:  %s", tmp_print_str_1_.c_str());

  // Longest Common Substring with Gaps
  auto [indiciesA, indiciesB] = lcs_indicies_(
                            comparable_words, comparable_words_new, allowed_gaps);
  if (indiciesA.empty()) {
    RCLCPP_INFO(node_ptr_->get_logger(), "  ---No overlap");
    transcript_.push_back(words_and_segments);
    return;
  }
  
  // Merge segments
  std::vector<Transcript::Operation> pending_ops;
  int op_offset = 0; // Increment when operation inerts into the array
  auto prevA = indiciesA[0], prevB = indiciesB[0];
  for(size_t i = 1; i <= indiciesA.size(); ++i) {
    RCLCPP_INFO(node_ptr_->get_logger(), "\tPrevA: %d,  PrevB:  %d", prevA, prevB);
    // Increment likely-hood of matched word in transcript
    pending_ops.push_back({Transcript::OperationType::INCREMENT, prevA, -1, op_offset});

    // Current index "i" may not be valid
    if (i == indiciesA.size()) {
      break;
    }
    auto next_idxA = indiciesA[i], next_idxB = indiciesB[i];
    auto curA = prevA + 1, curB = prevB + 1;
    RCLCPP_INFO(node_ptr_->get_logger(), "\tCurA: %d,  CurB:  %d", curA, curB);
    RCLCPP_INFO(node_ptr_->get_logger(), "\tNextA: %d,  NextB:  %d", next_idxA, next_idxB);

    // Conflicting/Different words
    while (curA != next_idxA && curB != next_idxB) {
      RCLCPP_INFO(node_ptr_->get_logger(), "\t\tLoop - CurA: %d,  CurB:  %d", curA, curB);
      RCLCPP_INFO(node_ptr_->get_logger(), "\t\tConflict Between '%s' and '%s'", 
                                                  transcript_.get_active_word(curA).c_str(), 
                                                  words[curB + skipped_ids_new[curB]].get().c_str());
      pending_ops.push_back({Transcript::OperationType::CONFLICT, curA, 
                                        curB + skipped_ids_new[curB], op_offset});
      curA++; curB++;
    }
    // New words appear in the update.  Insert them into the transcript
    while (curB != next_idxB) {
      RCLCPP_INFO(node_ptr_->get_logger(), "\t\tLoop - CurA: %d,  CurB:  %d", curA, curB);
      RCLCPP_INFO(node_ptr_->get_logger(), "\t\tInserting word '%s'  -- Between: '%s' and '%s'", 
                                                  words[curB + skipped_ids_new[curB]].get().c_str(),
                                                  transcript_.get_active_word(curA-1).c_str(),
                                                  transcript_.get_active_word(curA).c_str());
      pending_ops.push_back({Transcript::OperationType::INSERT, curA, 
                                                  curB + skipped_ids_new[curB], op_offset});
      op_offset++;
      curB++;
    }
    // Words in the transcript are not in update.  Decrease likelihood
    while (curA != next_idxA) {
      RCLCPP_INFO(node_ptr_->get_logger(), "\t\tLoop - CurA: %d,  CurB:  %d", curA, curB);
      RCLCPP_INFO(node_ptr_->get_logger(), "\t\tDecreasing Likelihood of word:  '%s'", 
                                                  transcript_.get_active_word(curA).c_str());
      pending_ops.push_back({Transcript::OperationType::DECREMENT, curA, -1, op_offset});
      curA++;
    }

    prevA = curA; prevB = curB;
  }

  // Handle words before indiciesB[0] in the new text
  // -- TODO
  // Insert the rest of words appearing in the new text on to the transcript
  prevB++;
  // while (prevB != static_cast<int>(words_and_segments.first.size())) {
  while (prevB != static_cast<int>(comparable_words_new.size())) {
    RCLCPP_INFO(node_ptr_->get_logger(), "\tCurA: %d,  CurB:  %d", prevA, prevB);
    RCLCPP_INFO(node_ptr_->get_logger(), "\t\tFINAL Inserting word '%s'  -- After: '%s'", 
                                words[prevB + skipped_ids_new[prevB]].get().c_str(),
                                transcript_.get_active_word(comparable_words.size()-1).c_str());
    pending_ops.push_back({Transcript::OperationType::INSERT, 
                            static_cast<int>(comparable_words.size()), 
                            prevB + skipped_ids_new[prevB], 
                            op_offset});
    op_offset++;
    prevB++;
  }

  // Apply operations
  for (const auto & op : pending_ops) {
   transcript_.run_op(op, words_and_segments);
  }
}
*/

void TranscriptManagerNode::clear_queue_() {
  bool one_merged = false;
  while ( !incoming_queue_->empty() ) {
    one_merged = true;
    auto words_and_segments = incoming_queue_->dequeue();
    // RCLCPP_INFO(node_ptr_->get_logger(), "Merging %ld words", words_and_segments.first.size());
    merge_one_(words_and_segments);
  }

  if (one_merged) {
    std::string print_str;
    auto message = AudioTranscript();
    auto words = transcript_.get_words();
    for (auto & word : words) {
      message.words.push_back(word.get());
      message.occ.push_back(word.get_occurrences());
      message.probs.push_back(1.);
      print_str += word.get();
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Current Transcript:   \n%s", print_str.c_str());

    message.active_index = transcript_.get_stale_word_id();
    transcript_pub_->publish(message);
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


std::tuple<std::vector<int>, std::vector<int>> TranscriptManagerNode::lcs_indicies_(
                                                  const std::vector<std::string>& textA,
                                                  const std::vector<std::string>& textB,
                                                  int allowedGaps) {
  int nA = textA.size();
  int nB = textB.size();

  // 2D DP table initialized to DPEntry(0, 0)
  std::vector<std::vector<DPEntry>> dp(nA + 1, std::vector<DPEntry>(nB + 1, {0, 0}));
  std::vector<std::vector<std::pair<int, int>>> 
                      prev(nA + 1, std::vector<std::pair<int, int>>(nB + 1, {-1, -1}));

  int maxLength = 0;
  int endIndexA = -1, endIndexB = -1;

  // Fill DP table
  for (int i = 1; i <= nA; ++i) {
    // RCLCPP_INFO(node_ptr_->get_logger(), "Word i: %s", textA[i-1].c_str());
    for (int j = 1; j <= nB; ++j) {
      // RCLCPP_INFO(node_ptr_->get_logger(), "\tWord j: %s", textB[j-1].c_str());
      if (textA[i-1] == textB[j-1]) {
        // RCLCPP_INFO(node_ptr_->get_logger(), "\t\tMatch");
        dp[i][j] = {dp[i-1][j-1].length + 1, 0};
        prev[i][j] = {i-1, j-1};
      } else {
        // Case 1: skip one element from textA
        if (dp[i-1][j].gaps < allowedGaps && dp[i][j].length < dp[i-1][j].length) {  
          // RCLCPP_INFO(node_ptr_->get_logger(), "\t\t Drop word from A");
          dp[i][j] = {dp[i-1][j].length, dp[i-1][j].gaps + 1};
          prev[i][j] = prev[i-1][j];
        }

        // Case 2: skip one element from textB
        if (dp[i][j-1].gaps < allowedGaps && dp[i][j].length < dp[i][j-1].length) {
          // RCLCPP_INFO(node_ptr_->get_logger(), "\t\t Drop word from B");
          dp[i][j] = {dp[i][j-1].length, dp[i][j-1].gaps + 1};
          prev[i][j] = prev[i][j-1];
        }

        // Case 3: skip one element from textA AND textB
        if (dp[i-1][j-1].gaps < allowedGaps && dp[i][j].length < dp[i-1][j-1].length) {
          // RCLCPP_INFO(node_ptr_->get_logger(), "\t\t Drop word from A AND B");
          dp[i][j] = {dp[i-1][j-1].length, dp[i-1][j-1].gaps + 1};
          prev[i][j] = prev[i-1][j-1];
        }
      }
      // RCLCPP_INFO(node_ptr_->get_logger(), "\t\tDP[%d][%d] Length: %d  Gap: %d", 
      //             i, j, dp[i][j].length, dp[i][j].gaps);

      // Track the maximum length
      if (dp[i][j].length >= maxLength) {
        maxLength = dp[i][j].length;
        endIndexA = i;
        endIndexB = j;
        // RCLCPP_INFO(node_ptr_->get_logger(), " ---- New Best Length: %d  endpoint A: %d   endpoint B: %d", 
        //             maxLength, endIndexA, endIndexB);
      }
    }
  }

  if (maxLength == 0) {
    return {{}, {}};
  }
  
  // Backtrack to find the longest matching subsequence
  std::string print_str = "Backtrack pairs: ";
  std::vector<int> resultA, resultB;
  std::tie(endIndexA, endIndexB) = prev[endIndexA][endIndexB];
  while (endIndexA != -1 && endIndexB != -1) {
    print_str += "(" + std::to_string(endIndexA) + "," + std::to_string(endIndexB) + "), ";
    resultA.push_back(endIndexA);  
    resultB.push_back(endIndexB);
    std::tie(endIndexA, endIndexB) = prev[endIndexA][endIndexB];
  }
  // RCLCPP_INFO(node_ptr_->get_logger(), "%s", print_str.c_str());

  std::reverse(resultA.begin(), resultA.end());
  std::reverse(resultB.begin(), resultB.end());

  return {resultA, resultB};
}


} // end of namespace whisper
