#include "transcript_manager/transcript.hpp"

namespace whisper {

bool Transcript::word_id_check_(const index &id, const std::vector<Segment> &segs, bool push_back) {
  // op.id_ + op_id_offset + stale_id == transcript_.size() when pushing back
  size_t extra = push_back ? 1 : 0;
  if ( seg_id_check_(id, segs) && 
      id.second >= 0 && static_cast<size_t>(id.second) < (segs[id.first].words_.size() + extra) ) {
    return true;
  }
  RCLCPP_WARN(node_ptr_->get_logger(), "Transcript op word id bounds check fail.");
  return false;
}

bool Transcript::seg_id_check_(const index &id, const std::vector<Segment>& segs, bool push_back) {
  size_t extra = push_back ? 1 : 0;
  if ( id.first >= 0 && static_cast<size_t>(id.first) < (segs.size() + extra) ) {
    return true;
  }
  RCLCPP_WARN(node_ptr_->get_logger(), "Transcript op segment id bounds check fail.");
  return false;
}

void Transcript::run(Operations &operations, const std::vector<Segment> &other) {
  // TODO:  Assert that operation ids increase (do not modify backwards)
  if ( operations.empty() ) { 
    return;
  }
  // Track insertions and deletions into the word/segment array and offset them
  //   The word offset tracker needs to be reset ever new segment.
  int word_offset = 0;
  int segment_offset = 0;
  int cur_segment = operations[0].id_.first;

  for (auto &op : operations) {

    // Offset seg and word insertions/deletions
    op.id_.first += segment_offset;
    if ( cur_segment != op.id_.first ) {
      word_offset = 0;
      cur_segment = op.id_.first;
    } 
    op.id_.second += word_offset;

    switch (op.op_type_) {
      case OperationType::INCREMENT:
        if ( !word_id_check_(op.id_, segments_) ) { continue; };
        inc_word_(op.id_);
        break;

      case OperationType::DECREMENT:
        if ( !word_id_check_(op.id_, segments_) ) { continue; };
        dec_word_(op.id_);
        break;

      case OperationType::INSERT:
        if ( !word_id_check_(op.id_, segments_, true) ) { continue; };
        if ( !word_id_check_(op.other_id_, other) ) { continue; };
        insert_word_(op.id_, other, op.other_id_);
        ++word_offset;
        break;

      case OperationType::DELETE:
        if ( !word_id_check_(op.id_, segments_) ) { continue; };
        del_word_(op.id_);
        --word_offset;
        break;

      case OperationType::CONFLICT:
        if ( !word_id_check_(op.id_, segments_) ) { continue; };
        if ( !word_id_check_(op.other_id_, other) ) { continue; };
        conflict_word_(op.id_, other, op.other_id_);
        break;

      case OperationType::INC_SEG:
        if ( !seg_id_check_(op.id_, segments_) ) { continue; };
        inc_segment_(op.id_);
        break;

      case OperationType::DEC_SEG:
        if ( !seg_id_check_(op.id_, segments_) ) { continue; };
        dec_segment_(op.id_);
        break;

      case OperationType::INSERT_SEG:
        ++op.id_.first; // Insert location
        insert_segment_(op.id_, other, op.other_id_);
        // Subtract remaining words from the old section to start the newly added section at 0
        word_offset -= (op.id_.first > 0 ? segments_[op.id_.first-1].words_.size() : 0);
        ++segment_offset;
        // Do not start a new segment
        cur_segment = op.id_.first;
        break;

      case OperationType::MERGE_SEG:
        if ( !seg_id_check_(op.id_, segments_) ) { continue; };
        if ( !seg_id_check_(op.other_id_, other) ) { continue; };
        merge_segments_(op.id_, other, op.other_id_);
        break;

      case OperationType::DEL_SEG:
        if ( !seg_id_check_(op.id_, segments_) ) { continue; };
        size_t words_in_segment = segments_[op.id_.first].words_.size();

        if ( del_segment_(op.id_) ) {
          // References to words in this segment will be (at the end) of the segment before
          word_offset += words_in_segment;
          --segment_offset;
          cur_segment = op.id_.first - 1;
        }
        break;
    }
  }

  // Remove any zero-length segments
  for (size_t seg_i = stale_segment_; seg_i < segments_.size(); ++seg_i) {
    if ( segments_[seg_i].words_.size() == 0 ) {
      del_segment_({static_cast<int>(seg_i), 0});
    }
  }
}

void Transcript::run(Operations &operations) {
  // Ensure all operations are valid
  for (const auto &op : operations) {
    if ( op.op_type_ == INSERT || 
          op.op_type_ == CONFLICT || 
          op.op_type_ == INSERT_SEG || 
          op.op_type_ == MERGE_SEG) {
      RCLCPP_WARN(node_ptr_->get_logger(), "Failed all (sub) operations!");
      return;
    }
  }
  run(operations, {});
}

void Transcript::inc_word_(const index id) {
  segments_[id.first].words_[id.second].inc_best();
}

void Transcript::dec_word_(const index id) {
  segments_[id.first].words_[id.second].dec_best();
}

void Transcript::del_word_(const index id) {
  auto& word_vec = segments_[id.first].words_;
  RCLCPP_DEBUG(node_ptr_->get_logger(), "[TSCRIPT OP] Deleting Word:  '%s'", 
                                            word_vec[id.second].get().c_str());
  
  word_vec.erase(word_vec.begin() + id.second);
}

void Transcript::inc_segment_(const index id) {
  segments_[id.first].inc();
}

void Transcript::dec_segment_(const index id) {
  segments_[id.first].dec();
}

void Transcript::insert_word_(const index id,
                              const std::vector<Segment> &other,
                              const index other_id) {
  auto& word_vec = segments_[id.first].words_;
  RCLCPP_DEBUG(node_ptr_->get_logger(), 
    "[TSCRIPT OP] Inserting '%s' in %s between '%s' -and- '%s'", 
    other[other_id.first].words_[other_id.second].get().c_str(),
    segments_[id.first].as_timestamp_str().c_str(),
    id.second == 0 ? "BEGIN" : word_vec[id.second-1].get().c_str(),
    id.second == static_cast<int>(word_vec.size()) ? "END" : word_vec[id.second].get().c_str());

  word_vec.insert(word_vec.begin() + id.second, other[other_id.first].words_[other_id.second]);
}

void Transcript::conflict_word_(const index id,
                                const std::vector<Segment> &other,
                                const index other_id) {
  RCLCPP_DEBUG(node_ptr_->get_logger(), 
                        "[TSCRIPT OP] Conflicting Words:  '%s' (:existing) v.s. (new:) '%s'", 
                        segments_[id.first].words_[id.second].get().c_str(),
                        other[other_id.first].words_[other_id.second].get().c_str());

  segments_[id.first].words_[id.second].compare(other[other_id.first].words_[other_id.second]);
}

bool Transcript::del_segment_(const index id) {
  RCLCPP_DEBUG(node_ptr_->get_logger(), "[TSCRIPT OP] Deleting Segment:  %s", 
                                            segments_[id.first].as_timestamp_str().c_str());

  auto& cur_seg = segments_[id.first];
  bool prev_seg_exists = id.first > 0 && (id.first-1) >= static_cast<int>(stale_segment_);
  bool post_seg_exists = id.first < static_cast<int>(segments_.size())-1;
  if ( !prev_seg_exists ) {
    if ( cur_seg.words_.empty() ) {
      // With no previous vector to add to, can only remove empty vectors
      segments_.erase(segments_.begin());
      return true;
    }
    RCLCPP_WARN(node_ptr_->get_logger(), "Attempt to delete segment failed");
    return false;
  }
  auto& prev_seg = segments_[id.first-1];

  // Move words from current segment to previous
  prev_seg.words_.insert(
      prev_seg.words_.end(),
      std::make_move_iterator(cur_seg.words_.begin()),
      std::make_move_iterator(cur_seg.words_.end()));
  
  // fix timestamps
  if ( post_seg_exists ) {
    prev_seg.set_duration_to(segments_[id.first+1]);
  } else {
    prev_seg.set_duration(prev_seg.get_duration() + cur_seg.get_duration());
  }

  // delete
  segments_.erase(segments_.begin() + id.first);
  return true;
}

bool Transcript::insert_segment_(const index id,
                                const std::vector<Segment> &other,
                                const index other_id) {
  RCLCPP_DEBUG(node_ptr_->get_logger(), 
                  "[TSCRIPT OP] Inserting Segment %s between %s -and- %s", 
                  other[other_id.first].as_timestamp_str().c_str(),
                  id.first == 0 ? "[BEGIN]" : segments_[id.first-1].as_timestamp_str().c_str(),
                  id.first == static_cast<int>(segments_.size()) ? 
                    "[END]" : segments_[id.first].as_timestamp_str().c_str());

  // If there is a segment before adjust the timestamp
  bool prev_seg_exists = id.first > 0 && (id.first-1) >= static_cast<int>(stale_segment_);
  bool post_seg_exists = id.first < static_cast<int>(segments_.size())-1;

  // Insert a blank segment with the new metadata, then move words from the previous segment
  segments_.insert(segments_.begin() + id.first, {other[other_id.first].data_});
  if ( prev_seg_exists ) {
    auto& prev_seg = segments_[id.first-1];

    if ( id.second >= 0 && static_cast<size_t>(id.second) < prev_seg.words_.size() ) {
      segments_[id.first].words_.insert(
            segments_[id.first].words_.begin(),
            std::make_move_iterator(prev_seg.words_.begin() + id.second),
            std::make_move_iterator(prev_seg.words_.end()));

      prev_seg.words_.erase(prev_seg.words_.begin() + id.second, prev_seg.words_.end());
      RCLCPP_DEBUG(node_ptr_->get_logger(), "\t-Created:  %s", segments_[id.first].as_str().c_str());
      RCLCPP_DEBUG(node_ptr_->get_logger(), "\t-Previous Segment:  %s", prev_seg.as_str().c_str());
    }
    
    // Adjust duration to current segment
    prev_seg.set_duration_to(other[other_id.first]);
  }

  // Adjust the duration of the inserted segment
  if ( post_seg_exists ) {
    segments_[id.first].set_duration_to(segments_[id.first+1]);
  }
  return true;
}

void Transcript::merge_segments_(const index id,
                                const std::vector<Segment> &other,
                                const index other_id) {
  RCLCPP_DEBUG(node_ptr_->get_logger(), 
                        "[TSCRIPT OP] Merging Segments:  %s (:existing) v.s. (new:) %s", 
                        segments_[id.first].as_timestamp_str().c_str(),
                        other[other_id.first].as_timestamp_str().c_str());

  bool prev_seg_exists = id.first > 0 && (id.first-1) >= static_cast<int>(stale_segment_);
  bool post_seg_exists = id.first < static_cast<int>(segments_.size())-1;
  segments_[id.first].overwrite(other[other_id.first]);
  segments_[id.first].inc();
  if ( prev_seg_exists ) {
    segments_[id.first-1].set_duration_to(segments_[id.first]);
  }
  if ( post_seg_exists ) {
    segments_[id.first].set_duration_to(segments_[id.first+1]);
  }
}

} // end of namespace whisper