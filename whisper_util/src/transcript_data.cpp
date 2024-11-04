#include "whisper_util/transcript_data.hpp"

namespace whisper {

void Transcript::push_back(const WordsAndSegments &words_and_segments) {
  const auto &[words, segments] = words_and_segments;
  for (size_t i = 0; i < words.size(); ++i) {
    transcript_.push_back(words[i]);
  }
  // for (size_t j = 0; j < words.size(); ++j) {
  //   segments_.push_back(segments[j]);
  // }
}

// Standard Operations
// void Transcript::run_op(const Operation operation, const WordsAndSegments &words_other) {
//   switch (operation.op_type_) {
//     case OperationType::INCREMENT:
//       inc_word(operation.id_ + operation.id_offset_);
//       break;
//     case OperationType::DECREMENT:
//       dec_word(operation.id_ + operation.id_offset_);
//       break;
//     case OperationType::INSERT:
//       insert_word(operation.id_ + operation.id_offset_, words_other, operation.other_id_);
//       break;
//     case OperationType::CONFLICT:
//       conflict_merge_word(operation.id_ + operation.id_offset_, words_other, operation.other_id_);
//       break;
//     default:
//       break;
//   }
// }

void Transcript::run(const Operations &operations, const WordsAndSegments &words_other) {
  // Increment when inserting, decrement when deleting from array
  int op_id_offset = 0; 
  for (const auto &op : operations) {
    switch (op.op_type_) {
      case OperationType::INCREMENT:
        inc_word(op.id_ + op_id_offset);
        break;
      case OperationType::DECREMENT:
        dec_word(op.id_ + op_id_offset);
        break;
      case OperationType::INSERT:
        insert_word(op.id_ + op_id_offset, words_other, op.other_id_);
        op_id_offset++;
        break;
      case OperationType::CONFLICT:
        conflict_merge_word(op.id_ + op_id_offset, words_other, op.other_id_);
        break;
      case OperationType::REMOVE:
        remove_word(op.id_ + op_id_offset);
        op_id_offset--;
        break;
      case OperationType::MATCHED_WORD:
        conflict_merge_word(op.id_ + op_id_offset, words_other, op.other_id_);
        inc_word(op.id_ + op_id_offset);
        break;
      case OperationType::MERGE_SEGMENTS:
        merge_word_segments(op.id_ + op_id_offset, words_other, op.other_id_);
        inc_word(op.id_ + op_id_offset);
        break;
    }
  }
}

void Transcript::run(const Operations &operations) {
  int op_id_offset = 0; 
  for (const auto &op : operations) {
    switch (op.op_type_) {
      case OperationType::INCREMENT:
        inc_word(op.id_ + op_id_offset);
        break;
      case OperationType::DECREMENT:
        dec_word(op.id_ + op_id_offset);
        break;
      case OperationType::REMOVE:
        remove_word(op.id_ + op_id_offset);
        op_id_offset--;
        break;
      default:
        break;
    }
  }
}




void Transcript::insert_word(const int id,
                const WordsAndSegments &new_words,
                const int new_word_id) {
  transcript_.insert(transcript_.begin() + id + stale_id_, new_words.first[new_word_id]);
  // segments_.insert(segments_.begin() + id + stale_id_, new_words.second[new_word_id]);
}

void Transcript::inc_word(const int id) {
  transcript_[id + stale_id_].inc_best();
}

void Transcript::dec_word(const int id) {
  transcript_[id + stale_id_].dec_best();
}

void Transcript::conflict_merge_word(const int id,
                  const WordsAndSegments &no_conflict_other,
                  const int other_id) {
  // auto word_other = no_conflict_other.first[other_id];
  // transcript_[id + stale_id_].compare(word_other.get_best_tokens(), word_other.get());
  transcript_[id + stale_id_].compare(no_conflict_other.first[other_id]);
}

void Transcript::remove_word(const int id) {
  printf("Removal of id:  %d -- (stale id:  %d)\n", id,
                                      stale_id_);
  transcript_.erase(transcript_.begin() + id + stale_id_);
}


void Transcript::merge_word_segments(const int id,
                  const WordsAndSegments &no_conflict_other,
                  const int other_id) {
  // auto word_other = no_conflict_other.first[other_id];
  // transcript_[id + stale_id_].compare(word_other.get_best_tokens(), word_other.get());
  transcript_[id + stale_id_].merge_segments(no_conflict_other.first[other_id]);
}


void Transcript::clear_mistakes(const int occurrence_threshold) {
    Transcript::Operations pending_ops;
    for (int id = stale_id_; id < static_cast<int>(transcript_.size()); ++id) {
      if (transcript_[id].get_occurrences() <= occurrence_threshold) {
        printf("Removed:  %s (%d)\n", transcript_[id].get().c_str(),
                                      transcript_[id].get_occurrences());
        pending_ops.push_back({REMOVE, id-stale_id_});
      }
    }
    run(pending_ops);
  }

} // end of namespace whisper
