#include "transcript_manager/transcript.hpp"

namespace whisper {

void Transcript::push_back(const std::vector<Word> &words_and_segments) {
  Transcript::Operations pending_ops;
  for (size_t i = 0; i < words_and_segments.size(); ++i) {
    pending_ops.push_back({INSERT, static_cast<int>(transcript_.size()) - stale_id_, 
                                                              static_cast<int>(i)});
  }
  run(pending_ops, words_and_segments);
}

void Transcript::clear_mistakes(const int occurrence_threshold) {
  Transcript::Operations pending_ops;
  for (int id = stale_id_; id < static_cast<int>(transcript_.size()); ++id) {
    if (transcript_[id].get_occurrences() <= occurrence_threshold) {
      pending_ops.push_back({DELETE, id-stale_id_});
    }
  }
  run(pending_ops);
}

std::string Transcript::get_print_str() {
  std::string print_str = "\033[34m";

  for (size_t i = 0; i < transcript_.size(); ++i) {
    const auto &word = transcript_[i];

    if ( word.is_segment() ) {
      print_str += "\n";  
      print_str += word.as_str();
    } else {
      print_str += word.get();
    }
  }

  print_str += "\033[0m";
  return print_str;
}

void Transcript::print_segment_ids() {
  bool first_print=true;
  printf("Segment ids:  ");
  for (const auto &s : segment_ids) {
    if (!first_print)
      printf(", ");
    printf("%d", s);
    first_print=false;
  }
  printf("\n");
}

} // end of namespace whisper
