#ifndef WHISPER_UTIL__TRANSCRIPT_HPP_
#define WHISPER_UTIL__TRANSCRIPT_HPP_

#include <vector>
#include <string>
#include <numeric>               // std::accumulate
#include <sstream>               // std::stringstream
#include <iomanip>               // std::put_time, std::setfill
#include <algorithm>             // std::remove_if

#include "rclcpp/rclcpp.hpp"     // node_ptr_ (only used for logging)

#include "whisper_util/chrono_utils.hpp"
#include "transcript_manager/words.hpp"
#include "transcript_manager/segments.hpp"

namespace whisper {

/**
 * @brief Transcript keeps a vector of segments and allows operations/and merging. 
 * Also track at what point the transcript is stale and no longer updated.
 */
class Transcript {
private:
  std::vector<Segment> segments_;
  size_t stale_segment_;

  // LCS Hyperparameter
  int allowed_gaps_;

  // Only used for logging
  rclcpp::Node::SharedPtr node_ptr_;

public:
  using index = std::pair<int, int>; // first -- segment, second -- word

  // transcript_operations.cpp
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
  void inc_word_(const index id);
  void dec_word_(const index id);
  void del_word_(const index id);
  void insert_word_(const index id, const std::vector<Segment> &other, const index other_id);
  void conflict_word_(const index id, const std::vector<Segment> &other, const index other_id);

  void inc_segment_(const index id);
  void dec_segment_(const index id);
  bool del_segment_(const index id);
  bool insert_segment_(const index id, const std::vector<Segment> &other,const index other_id);
  void merge_segments_(const index id, const std::vector<Segment> &other, const index other_id);

  // Helper functions
  bool seg_id_check_(const index &id, const std::vector<Segment>& segs, bool push_back = false);
  bool word_id_check_(const index &id, const std::vector<Segment>& segs, bool push_back = false);

public:
  Transcript(const int allowed_gaps, const rclcpp::Node::SharedPtr node_ptr): 
              stale_segment_(0), allowed_gaps_(allowed_gaps), node_ptr_(node_ptr) {};

  // transcript.cpp
  void push_back(const std::vector<Segment> &other);
  void clear_mistakes(const int occurrence_threshold);
  std::string get_print_str();
  // every segment starting before time_thresh will no longer be altered
  void set_stale_segment(std::chrono::system_clock::time_point time_thresh);

  // Utilities:
  inline bool empty() const { return segments_.empty(); }
  inline void clear() { segments_.clear(); stale_segment_ = 0; };
  inline size_t get_stale_segment() const { return stale_segment_; };
  inline size_t size() const { return segments_.size(); };

  // Provide access to the const iterator
  using const_seg_iterator = typename std::vector<Segment>::const_iterator;
  const_seg_iterator segments_begin() const { return segments_.cbegin(); }
  const_seg_iterator segments_end() const { return segments_.cend(); }

  // transcript_algorithms.cpp
  void merge_one(const std::vector<Segment> &other);

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
