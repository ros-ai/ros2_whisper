#include "transcript_manager/transcript.hpp"

namespace whisper {

void Transcript::run(const Operations &operations, const std::vector<Word> &words_other) {
  // Increment when inserting, decrement when deleting from array
  int op_id_offset = 0; 
  for (const auto &op : operations) {
    switch (op.op_type_) {
      case OperationType::INCREMENT:
        if ( !id_check(op.id_ + op_id_offset + stale_id_, transcript_.size()) ) { continue; };
        inc_word_or_seg(op.id_ + op_id_offset);
        break;

      case OperationType::DECREMENT:
        if ( !id_check(op.id_ + op_id_offset + stale_id_, transcript_.size()) ) { continue; };
        dec_word_or_seg(op.id_ + op_id_offset);
        break;

      case OperationType::INSERT:
        // op.id_ + op_id_offset + stale_id == transcript_.size() when pushing back
        if ( !id_check(op.id_ + op_id_offset + stale_id_, transcript_.size()+1) ) { continue; };
        if ( !id_check(op.other_id_, words_other.size()) ) { continue; };

        if ( is_seg(op.other_id_, words_other) ) {
          insert_segment(op.id_ + op_id_offset, words_other, op.other_id_);
        } else {
          insert_word(op.id_ + op_id_offset, words_other, op.other_id_);
        }
        op_id_offset++;
        break;

      case OperationType::DELETE:
        if ( !id_check(op.id_ + op_id_offset + stale_id_, transcript_.size()) ) { continue; };

        if ( is_seg(op.id_ + op_id_offset + stale_id_, transcript_) ) {
          del_segment(op.id_ + op_id_offset);
        } else {
          del_word(op.id_ + op_id_offset);
        }
        
        op_id_offset--;
        break;

      case OperationType::MERGE:
      case OperationType::CONFLICT:
        if ( !id_check(op.id_ + op_id_offset + stale_id_, transcript_.size()) ) { continue; };
        if ( !id_check(op.other_id_, words_other.size()) ) { continue; };
        bool is_seg_a = is_seg(op.id_ + op_id_offset + stale_id_, transcript_);
        bool is_seg_b = is_seg(op.other_id_, words_other);

        if ( is_seg_a && is_seg_b ) {
          merge_segments(op.id_ + op_id_offset, words_other, op.other_id_);
        } else if ( !is_seg_a && !is_seg_b ) {
          conflict_word(op.id_ + op_id_offset, words_other, op.other_id_);
        } else {
          RCLCPP_WARN(node_ptr_->get_logger(), "Failed attempting to merge word and segment together.");
        }
        break;
    }
  }
}

void Transcript::run(const Operations &operations) {
  int op_id_offset = 0; 
  for (const auto &op : operations) {
    if ( !id_check(op.id_ + op_id_offset + stale_id_, transcript_.size()) ) { continue; };

    switch (op.op_type_) {
      case OperationType::INCREMENT:
        inc_word_or_seg(op.id_ + op_id_offset);
        break;

      case OperationType::DECREMENT:
        dec_word_or_seg(op.id_ + op_id_offset);
        break;

      case OperationType::DELETE:
        if ( transcript_[op.id_ + op_id_offset + stale_id_].is_segment() ) {
          del_segment(op.id_ + op_id_offset);
        } else {
          del_word(op.id_ + op_id_offset);
        }
        op_id_offset--;
        break;

      default:
        break;
    }
  }
}

// 
// Standard Operations
// 
void Transcript::inc_word_or_seg(const int id) {
  transcript_[id + stale_id_].inc_best();
}

void Transcript::dec_word_or_seg(const int id) {
  transcript_[id + stale_id_].dec_best();
}

void Transcript::del_word(const int id) {
  transcript_.erase(transcript_.begin() + id + stale_id_);

  // Decrement future segment pointers
  auto it = std::lower_bound(segment_ids.begin(), segment_ids.end(), id + stale_id_);
  while (it != segment_ids.end()) {
    --*it;
    ++it;
  }
}

void Transcript::insert_word(const int id,
                              const std::vector<Word> &new_words,
                              const int new_word_id) {
  transcript_.insert(transcript_.begin() + id + stale_id_, new_words[new_word_id]);
  
  // Increment pointer (>=) to id + stale_id_.
  auto it = std::lower_bound(segment_ids.begin(), segment_ids.end(), id + stale_id_);
  while (it != segment_ids.end()) {
    ++*it; // inc to match insert
    ++it;  // move to next segment
  }
}

void Transcript::conflict_word(const int id,
                  const std::vector<Word> &new_words,
                  const int other_id) {
  transcript_[id + stale_id_].compare(new_words[other_id]);
}

void Transcript::del_segment(const int id) {
  auto it = std::lower_bound(segment_ids.begin(), segment_ids.end(), id + stale_id_);
  bool prev_seg_exists = it != segment_ids.begin();
  bool post_seg_exists = it != segment_ids.end() && (it+1) != segment_ids.end();
  int prev, post;
  if ( prev_seg_exists ) {
    prev_seg_exists = 
            is_valid_start(transcript_[*(it-1)].get_start(), transcript_[*(it)].get_start());
    prev = *(it-1);
  }
  if ( post_seg_exists ) {
    post_seg_exists = 
            is_valid_start(transcript_[*(it)].get_start(), transcript_[*(it+1)].get_start());
    post = *(it+1);
  }

  if ( prev_seg_exists ) {
    if ( !post_seg_exists ) {
      RCLCPP_DEBUG(node_ptr_->get_logger(), 
            "Extending duration of %d (abs) to:  %ld (ms) -> %ld (ms)", prev,
            transcript_[prev].get_duration().count(),
            (transcript_[prev].get_duration() + transcript_[*it].get_duration()).count());

      transcript_[prev].set_duration(transcript_[prev].get_duration() + transcript_[*it].get_duration());
    } else {
      RCLCPP_DEBUG(node_ptr_->get_logger(), 
            "Segment middle deletion, extending duration of ids from [%d '%s'] -to- [%d '%s'] "
            " with prev value change:  %ld (ms) -> %ld (ms)", 
            prev, transcript_[prev].as_timestamp_str().c_str(),
            post, transcript_[post].as_timestamp_str().c_str(),
            transcript_[prev].get_duration().count(),
            std::chrono::duration_cast<std::chrono::milliseconds>(
            transcript_[post].get_start() - transcript_[prev].get_start()).count());

      transcript_[prev].set_duration(std::chrono::duration_cast<std::chrono::milliseconds>(
                                transcript_[post].get_start() - transcript_[prev].get_start()));
    }
  } else {
    // Do nothing -- No previous segment to extend when current segment is deleted
  }
  segment_ids.erase(it); 
  del_word(id); 
}

void Transcript::insert_segment(const int id,
                  const std::vector<Word> &new_words,
                  const int new_word_id) {
  auto it = std::lower_bound(segment_ids.begin(), segment_ids.end(), id + stale_id_);
  bool prev_seg_exists = it != segment_ids.begin() && segment_ids.size() > 0;
  bool post_seg_exists = it != segment_ids.end();
  std::chrono::milliseconds prev_duration, cur_duration;
  if ( prev_seg_exists ) {
    prev_seg_exists = 
            is_valid_start(transcript_[*(it-1)].get_start(), new_words[new_word_id].get_start());
    prev_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                        new_words[new_word_id].get_start() - transcript_[*(it-1)].get_start());
  }
  if ( post_seg_exists ) {
    post_seg_exists = 
            is_valid_start(new_words[new_word_id].get_start(), transcript_[*(it)].get_start());
    cur_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                      transcript_[*(it)].get_start() - new_words[new_word_id].get_start());
  }

  // Run Insert 
  insert_word(id, new_words, new_word_id);
  if ( prev_seg_exists ) {
    RCLCPP_DEBUG(node_ptr_->get_logger(),
      "Adjusting ealier segment %d (abs) '%s', duration values:  %ld (ms) -> %ld (ms)", 
                                                      *(it-1),
                                                      transcript_[*(it-1)].as_timestamp_str().c_str(), 
                                                      transcript_[*(it-1)].get_duration().count(),
                                                      prev_duration.count());
    
    transcript_[*(it-1)].set_duration(prev_duration);
  }
  if ( post_seg_exists ) {
    RCLCPP_DEBUG(node_ptr_->get_logger(),
      "\tAdjusting current segment %d (abs) '%s', duration values:  %ld (ms) -> %ld (ms)", 
                                              id + stale_id_,
                                              transcript_[id + stale_id_].as_timestamp_str().c_str(), 
                                              transcript_[id + stale_id_].get_duration().count(),
                                              cur_duration.count());
    
    transcript_[id + stale_id_].set_duration(cur_duration);
  }
  // Add id to segments array
  segment_ids.insert(it, id + stale_id_);
}

void Transcript::merge_segments(const int id,
                  const std::vector<Word> &new_words,
                  const int other_id) {
  auto it = std::lower_bound(segment_ids.begin(), segment_ids.end(), id + stale_id_);
  if (*it != (id +stale_id_)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Error segment id not found in segment id array.");
    return;
  }

  bool prev_seg_exists = it != segment_ids.begin();
  bool post_seg_exists = it != segment_ids.end() && (it+1) != segment_ids.end();
  std::chrono::milliseconds new_prev_duration, new_cur_duration;
  if ( prev_seg_exists ) {
    if ( !is_valid_start(transcript_[*(it-1)].get_start(), new_words[other_id].get_start()) ) {
      new_prev_duration = std::chrono::milliseconds(0);
    } else {
      new_prev_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                        new_words[other_id].get_start() - transcript_[*(it-1)].get_start());
    }
  }
  if ( post_seg_exists ) {
    if ( !is_valid_start(new_words[other_id].get_start(), transcript_[*(it+1)].get_start()) ) {
      new_cur_duration = std::chrono::milliseconds(0);
    } else {
      new_cur_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
              transcript_[*(it+1)].get_start() - new_words[other_id].get_start());
    }
  }

  if ( prev_seg_exists ) {
    RCLCPP_DEBUG(node_ptr_->get_logger(), 
                      "Adjusting previous (%d, '%s') from:  %ld (ms) -> %ld (ms)", 
                                        *(it-1),
                                        transcript_[*(it-1)].as_timestamp_str().c_str(), 
                                        transcript_[*(it-1)].get_duration().count(), 
                                        new_prev_duration.count());
    transcript_[*(it-1)].set_duration(new_prev_duration);
  }

  // Overwrite data before fixing the duration
  transcript_[id + stale_id_].overwrite(new_words[other_id]);
  transcript_[id + stale_id_].inc_best();

  if ( post_seg_exists ) {
    RCLCPP_DEBUG(node_ptr_->get_logger(), 
                      "Adjusting cur (%d, '%s') from:  %ld (ms) -> %ld (ms)", 
                                        *(it),
                                        transcript_[id + stale_id_].as_timestamp_str().c_str(), 
                                        transcript_[id + stale_id_].get_duration().count(), 
                                        new_cur_duration.count());
    transcript_[id + stale_id_].set_duration(new_cur_duration);
  }
}

bool Transcript::is_valid_start(std::chrono::system_clock::time_point prev, 
                      std::chrono::system_clock::time_point next) {
  auto valid = prev <= next;
  if ( !valid ) {
    RCLCPP_WARN(node_ptr_->get_logger(), "Invalid Start.   Previous (%s) after Next (%s)",
                              timestamp_as_str(prev).c_str(), timestamp_as_str(next).c_str());
  }
  return valid;
}

} // end of namespace whisper
