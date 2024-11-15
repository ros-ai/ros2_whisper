#ifndef WHISPER_UTIL__TRANSCRIPT_HPP_
#define WHISPER_UTIL__TRANSCRIPT_HPP_

#include <vector>
#include <string>
#include <numeric> // std::accumulate
#include <sstream> // std::stringstream
#include <iomanip>  // std::put_time, std::setfill
#include <algorithm> // std::remove_if

#include "rclcpp/rclcpp.hpp" // node_ptr_

#include "whisper_util/chrono_utils.hpp"
#include "transcript_manager/words.hpp"
#include "transcript_manager/segments.hpp"

namespace whisper {

// Declare Helper Functions
inline bool is_special_token(const std::vector<std::string> &tokens, const int idx);
inline bool my_ispunct(const std::vector<std::string> &tokens, const int idx);
inline std::pair<bool, int> join_tokens(const std::vector<std::string> &tokens, const int idx);
inline std::string combine_text(const std::vector<std::string> &tokens, 
                                                            const int idx, const int num);
inline float combine_prob(const std::vector<float> &probs, const int idx, const int num);

/**
 * @brief Perform operations over a vector of words.
 */
class Transcript {
private:
  std::vector<Segment> segments_;
  size_t stale_segment_;
  // Hash a word index to a specific segment.
  // size_t word_so_far;
  // Use the count up to that segment to get the word offset. seg_counts.size() == segments_.size()
  // std::vector<size_t> seg_counts_;

  // inline size_t words_so_far() const { return word_so_far; };
  // inline size_t get_stale_id_() const { 
  //   return stale_segment_ < seg_counts_.size() ? seg_counts_[stale_segment_] : 0; 
  // };

  // LCS Hyperparameter
  int allowed_gaps_;

  // Primaraly used for logging
  rclcpp::Node::SharedPtr node_ptr_;

public:
  using index = std::pair<int, int>; // first -- segment, second -- word
  // Implmentation in transcript_operations.cpp
  enum OperationType {INCREMENT, DECREMENT, INSERT, DELETE, CONFLICT, 
                        INC_SEG, DEC_SEG, INSERT_SEG, DEL_SEG, MERGE_SEG};
  struct Operation {
    const OperationType op_type_;
    index id_;
    const index other_id_;

    Operation(const OperationType op_type, const index id) : 
                    op_type_(op_type), id_(id), other_id_({0, 0}) {
      if ( !(op_type_ == INCREMENT || op_type_ == DECREMENT || op_type_ == DELETE ||
              op_type_ == INC_SEG || op_type_ == DEC_SEG || op_type_ == DEL_SEG) ) {
        throw std::runtime_error("Missing argument on Operation initialization.");
      }
    }
    Operation(const OperationType op_type, const index id, const index other_id) : 
                  op_type_(op_type), id_(id), other_id_(other_id) {}
  }; // struct Operation
  using Operations = std::vector<Operation>;
  void run(Operations &operations, const std::vector<Segment> &other);    // run all ops
  void run(Operations &operations);                                       // run subset

private:
  void inc_word(const index id);
  void dec_word(const index id);
  void del_word(const index id);
  void insert_word(const index id, const std::vector<Segment> &other, const index other_id);
  void conflict_word(const index id, const std::vector<Segment> &other, const index other_id);

  void inc_segment(const index id);
  void dec_segment(const index id);
  bool del_segment(const index id);
  bool insert_segment(const index id, const std::vector<Segment> &other,const index other_id);
  void merge_segments(const index id, const std::vector<Segment> &other, const index other_id);

  // Helper functions
  bool seg_id_check(const index &id, const std::vector<Segment>& segs, bool push_back = false);
  bool word_id_check(const index &id, const std::vector<Segment>& segs, bool push_back = false);

public:
  Transcript(const int allowed_gaps, const rclcpp::Node::SharedPtr node_ptr): 
              stale_segment_(0), allowed_gaps_(allowed_gaps), node_ptr_(node_ptr) {};

  void push_back(const std::vector<Segment> &other);
  void clear_mistakes(const int occurrence_threshold);
  std::string get_print_str();
  void merge_one_(const std::vector<Segment> &other);

  bool empty() const { return segments_.empty(); }
  size_t get_stale_seg_id() const { return stale_segment_; }

  // every segment starting before time_thresh will no longer be altered
  void set_stale_segment(std::chrono::system_clock::time_point time_thresh);
  void clear() { segments_.clear(); stale_segment_ = 0; };
  size_t get_stale_segment() const { return stale_segment_; };
  size_t size() const { return segments_.size(); };
  
  // void set_stale_word_id(const int stale_id) { stale_id_ = stale_id; }
  // Set the stale section to the current time minus ... seconds
  // void set_stale_word_id();

  // std::vector<Word> get_words() {
  //   // TODO:  return iterator
  //   return transcript_;
  // }

  // std::vector<Word> get_words_splice() {
  //   // TODO:  return iterator
  //   return std::vector<Word>(transcript_.begin() + stale_id_, transcript_.end());
  // }

  // Provide access to the const iterator
  using const_seg_iterator = typename std::vector<Segment>::const_iterator;
  const_seg_iterator segments_begin() const { return segments_.cbegin(); }
  const_seg_iterator segments_end() const { return segments_.cend(); }

// 
// Iterators
// 
public:
//   class Iterator {
//   public:
//     using iterator_category = std::forward_iterator_tag;
//     using value_type = Word;
//     using difference_type = std::ptrdiff_t;
//     using ptr = value_type*; using cptr = const value_type*;
//     using ref = value_type&; using cref = const value_type&;
    
//     Iterator(Transcript* transcript, size_t seg_id)
//             : transcript_(transcript), 
//               seg_id_(std::min(seg_id, transcript->segments_.size())), word_id_(0) {
//       abs_id = seg_id < transcript_->seg_counts_.size() ? 
//                         transcript_->seg_counts_[seg_id_] : transcript_->words_so_far();
//       rel_id = transcript_->stale_segment_ <= seg_id ? abs_id - transcript_->get_stale_id_() : 0;
//     }
//     ref operator*() { return transcript_->segments_[seg_id_].words_[word_id_]; }
//     cref operator*() const { return transcript_->segments_[seg_id_].words_[word_id_]; }
//     ptr operator->() { return &transcript_->segments_[seg_id_].words_[word_id_]; }
//     cptr operator->() const { return &transcript_->segments_[seg_id_].words_[word_id_]; }

//     Iterator& operator++() {
//       ++word_id_;
//       ++abs_id;
//       if ( seg_id_ >= transcript_->stale_segment_ ) {
//         ++rel_id;
//       }
//       advance_to_valid_position_();
//       return *this;
//     }
//     Iterator operator++(int) {
//       Iterator temp = *this;
//       ++(*this);
//       return temp;
//     }

//     bool operator==(const Iterator& other) const {
//       return seg_id_ == other.seg_id_ && word_id_ == other.word_id_;
//     }
//     bool operator!=(const Iterator& other) const { return !(*this == other); }

//     // Get the word id relative to active words
//     size_t get_id_rel() const { return rel_id; }
//     size_t get_id_abs() const { return abs_id; }

//   private:
//     Transcript* transcript_;
//     size_t seg_id_;
//     size_t word_id_;

//     // Computed
//     size_t abs_id;  // id from all words in transcript
//     size_t rel_id;  // id relative to active words 

//     void advance_to_valid_position_() {
//       while (seg_id_ < transcript_->segments_.size() && 
//               word_id_ >= transcript_->segments_[seg_id_].words_.size()) {
//         ++seg_id_;
//         word_id_ = 0;
//       }
//     }
//   }; // class Iterator

// Iterator begin() { return Iterator(this, 0); }
// Iterator begin_active() { return Iterator(this, stale_segment_); }
// Iterator end() { return Iterator(this, segments_.size()); }
// End of Iterators

private:
  // Longest common substring (allowing gaps for conflicting words)
  struct DPEntry {
      int length;
      int gaps;
  };
  std::tuple<std::vector<int>, std::vector<int>> lcs_indicies_(
                                                  const std::vector<std::string>& textA,
                                                  const std::vector<std::string>& textB,
                                                  int allowedGaps);
};

} // end of namespace whisper
#endif // WHISPER_UTIL__TRANSCRIPT_HPP_
