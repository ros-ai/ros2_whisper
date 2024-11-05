#include "whisper_util/transcript_data.hpp"

namespace whisper {

void Transcript::run(const Operations &operations, const std::vector<Word> &words_other) {
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

// Standard Operations
void Transcript::inc_word(const int id) {
  transcript_[id + stale_id_].inc_best();
}

void Transcript::dec_word(const int id) {
  transcript_[id + stale_id_].dec_best();
}

void Transcript::remove_word(const int id) {
  transcript_.erase(transcript_.begin() + id + stale_id_);
}

void Transcript::insert_word(const int id,
                const std::vector<Word> &new_words,
                const int new_word_id) {
  transcript_.insert(transcript_.begin() + id + stale_id_, new_words[new_word_id]);
}

void Transcript::conflict_merge_word(const int id,
                  const std::vector<Word> &no_conflict_other,
                  const int other_id) {
  transcript_[id + stale_id_].compare(no_conflict_other[other_id]);
}

void Transcript::merge_word_segments(const int id,
                  const std::vector<Word> &no_conflict_other,
                  const int other_id) {
  transcript_[id + stale_id_].merge_segments(no_conflict_other[other_id]);
}

// Other Functions
void Transcript::push_back(const std::vector<Word> &words_and_segments) {
  for (size_t i = 0; i < words_and_segments.size(); ++i) {
    transcript_.push_back(words_and_segments[i]);
  }
}

void Transcript::clear_mistakes(const int occurrence_threshold) {
  Transcript::Operations pending_ops;
  for (int id = stale_id_; id < static_cast<int>(transcript_.size()); ++id) {
    if (transcript_[id].get_occurrences() <= occurrence_threshold) {
      pending_ops.push_back({REMOVE, id-stale_id_});
    }
  }
  run(pending_ops);
}

} // end of namespace whisper
