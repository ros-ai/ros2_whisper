#include "transcript_manager/transcript.hpp"

namespace whisper {

void Transcript::push_back(const std::vector<Segment> &other) {
  Transcript::Operations pending_ops;
  int last_seg = static_cast<int>(segments_.size()) - 1;
  int last_word = last_seg >= 0 ? segments_[last_seg].words_.size() : 0;
  index last_id{last_seg, last_word};
  for (size_t seg_i_o = 0; seg_i_o < other.size(); ++seg_i_o) {
    pending_ops.push_back({INSERT_SEG, last_id, {seg_i_o, 0}});

    for (size_t word_i_o = 0; word_i_o < other[seg_i_o].words_.size(); ++word_i_o) {
      pending_ops.push_back({INSERT, last_id, {seg_i_o, word_i_o}});
    }
  }
  run(pending_ops, other);
}

void Transcript::clear_mistakes(const int occurrence_threshold) {
  Transcript::Operations pending_ops;
  for (size_t seg_i = stale_segment_; seg_i < segments_.size(); ++seg_i) {
    // Just delete boundary and move words to previous segment
    if ( segments_[seg_i].occ <= occurrence_threshold ) {
      pending_ops.push_back({DEL_SEG, {seg_i, 0}});
    }
    // Remove low liklihood words from the transcript
    for (size_t word_i = 0; word_i < segments_[seg_i].words_.size(); ++word_i) {
      const auto& word = segments_[seg_i].words_[word_i];
      if ( word.get_occurrences() <= occurrence_threshold ) {
        pending_ops.push_back({DELETE, {seg_i, word_i}});
      }
    }
  }
  run(pending_ops);
}


void Transcript::set_stale_segment(std::chrono::system_clock::time_point time_thresh) {
  int new_stale_segment_ = stale_segment_;
  for (size_t seg_i = stale_segment_; seg_i < segments_.size(); ++seg_i) {
    if ( !(segments_[seg_i].get_start() < time_thresh) ) {
      // Stale id will be last segment
      break;
    }
    new_stale_segment_ = seg_i;
  }
  stale_segment_ = new_stale_segment_;
}


std::string Transcript::get_print_str() {
  std::string print_str = "\033[34m";
  bool first_print = true;
  for (const auto& seg : segments_) {
    if ( !first_print ) {
      print_str += "\n";
    }
    print_str += seg.as_str();
    first_print = false;
  }
  print_str += "\033[0m";
  return print_str;
}

} // end of namespace whisper
