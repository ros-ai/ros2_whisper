#ifndef WHISPER_UTIL__TRANSCRIPT_HPP_
#define WHISPER_UTIL__TRANSCRIPT_HPP_

#include <vector>
#include <string>
#include <numeric> // std::accumulate
#include <sstream> // std::stringstream
#include <iomanip>  // std::put_time, std::setfill
#include <algorithm> // std::remove_if

#include "transcript_manager/words.hpp"

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
  std::vector<Word> transcript_;
  std::vector<int> segment_ids;      // Sorted Array of transcript_ indicies
  int stale_id_;

public:
  enum OperationType {INCREMENT, DECREMENT, INSERT, DELETE, MERGE, CONFLICT};
  struct Operation {
    const OperationType op_type_;
    const int id_;
    const int other_id_;

    Operation(const OperationType op_type, const int id) : 
                    op_type_(op_type), id_(id), other_id_(-1) {
      if ( !(op_type_ == INCREMENT || op_type_ == DECREMENT || op_type_ == DELETE) ) {
        throw std::runtime_error("Missing argument on Operation initialization.");
      }
    }
    Operation(const OperationType op_type, const int id, const int other_id) : 
                  op_type_(op_type), id_(id), other_id_(other_id) {}
  };
  using Operations = std::vector<Operation>;

  void inc_word_or_seg(const int id);
  void dec_word_or_seg(const int id);

  void del_word(const int id);
  void insert_word(const int id, const std::vector<Word> &new_words,
                    const int new_word_id);
  void conflict_word(const int id, const std::vector<Word> &new_words,
                    const int other_id);

  void del_segment(const int id);
  void insert_segment(const int id, const std::vector<Word> &new_words,
                                                    const int new_word_id);
  void merge_segments(const int id, const std::vector<Word> &new_words,
                                                    const int new_word_id);

  void run(const Operations &operations, const std::vector<Word> &words_other);   // run all ops
  void run(const Operations &operations);                                         // run subset

public:
  Transcript(): stale_id_(0) {};

  // 
  // Other Trancript Functions
  // 
  void push_back(const std::vector<Word> &words_and_segments);

  bool empty() const {
    return transcript_.empty();
  }

  std::vector<Word> get_words() {
    return transcript_;
  }

  std::vector<Word> get_words_splice() {
    // TODO:  return iterator
    return std::vector<Word>(transcript_.begin() + stale_id_, transcript_.end());
  }

  // std::string get_active_word(const int id) {
  //   return transcript_[id + stale_id_].get();
  // }

  int get_stale_word_id() const {
    return stale_id_;
  }

  void set_stale_word_id(const int stale_id) {
    stale_id_ = stale_id;
  }

  void clear_mistakes(const int occurrence_threshold);

  // Provide access to the const iterator
  using const_iterator = typename std::vector<Word>::const_iterator;
  const_iterator begin() const { return transcript_.cbegin(); }
  const_iterator end() const { return transcript_.cend(); }

  std::string get_print_str() {
    std::string print_str = "\033[34m";
    // for (const auto & word : transcript_) {
    for (size_t i = 0; i < transcript_.size(); ++i) {
      const auto &word = transcript_[i];
      if ( word.is_segment() ) {
        print_str += "\n";  
        print_str += word.as_str();
        continue;
      }
      print_str += word.get();
    }
    print_str += "\033[0m";
    return print_str;
  }

private:
  inline bool id_check(const int &id, const size_t &max_val) {
    if ( id < 0 || static_cast<size_t>(id) >= max_val ) {
      fprintf(stderr, "Failed bounds check on run op. id: %d, max_val:  %ld\n", id, max_val);
      return false;
    }
    return true;
  }

  inline bool is_seg(const int &id, const std::vector<Word> &words) {
    return words[id].is_segment();
  }

  void print_segment_ids();
};

// 
// Helper Functions
//
inline bool is_special_token(const std::vector<std::string> &tokens, const int idx) {
  const std::vector<std::string> special_token_start_strs = {
      "[_BEG_]", "[_TT_", " [_BEG_]", " [_TT_"
  };
  if ( idx < 0 || tokens.size() <= static_cast<size_t>(idx) ) {
    return false;
  }
  for (const auto &start : special_token_start_strs) { 
    if ( tokens[idx].size() >= start.size() && tokens[idx].substr(0, start.size()) == start ) {
      return true;
    }
  }
  return false;
}

inline bool my_ispunct(const std::vector<std::string> &tokens, const int idx) {
  // The reason for not using std::punct(..) on the first character is that "'t" would 
  //    be considered a punctuation.
  // As well as some brackets which should be combined into words.
  // Otherwise it is missing "..." which we can consider as punctuation
  const std::vector<std::string> punctuations = {",", ".", "?", "!", ":", ";", "...", "+", "-"};
  if ( idx < 0 || tokens.size() <= static_cast<size_t>(idx) ) {
    return false;
  }
  return std::find(punctuations.begin(), punctuations.end(), tokens[idx]) != punctuations.end();
}

inline bool contains_char(const std::string &str, const char target) {
  for (const auto &c : str) {
    if ( target == c ) {
      return true;
    }
  }
  return false;
}

inline std::pair<bool, int> join_tokens(const std::vector<std::string> &tokens, const int idx) {
  // Check if, starting from the idx, the tokens are a bracket which can be combined or removed
  //   Return:
  //      bool -- Tokens start with bracket (can be combined)
  //      int  -- Number of tokens to combine
  const std::vector<std::pair<char, char>> combine_brackets = {
      {'[', ']'},
      {'{', '}'},
      {'(', ')'}
  };
  // Set a limit on how many tokens can be within brackets
  const int max_allowed_tokens_to_combine = 10;

  if ( idx < 0 || tokens.size() <= static_cast<size_t>(idx) ) {
    return {false, 0};
  }

  // Try to combine tokens
  for (auto &[start, end] : combine_brackets) { 
    if ( contains_char(tokens[idx], start) ) {
      int end_idx = idx + 1;
      while (end_idx < static_cast<int>(tokens.size()) && (end_idx-idx) 
                                            <= max_allowed_tokens_to_combine) {
        if ( contains_char(tokens[end_idx], end) ) {
          return {true, end_idx - idx + 1};
        }
        end_idx++;
      }
      // We found the start but not the end, "combine" current token with itself
      return {true, 1};
    }
  }
  return {false, 0};
}

inline std::string combine_text(const std::vector<std::string> &tokens, 
                                                      const int idx, const int num) {
  if ( idx < 0 || tokens.size() <= static_cast<size_t>(idx) ) {
    return "";
  }
  if ( num <= 0 || tokens.size() <= static_cast<size_t>(idx + num - 1) ) {
    return "";
  }
  std::string ret = "";
  for (auto i = idx; i < idx + num; ++i) {
    ret += tokens[i];
  }
  return ret;
}

inline float combine_prob(const std::vector<float> &probs, const int idx, const int num) {
  if ( idx < 0 || probs.size() <= static_cast<size_t>(idx) ) {
    return 0.;
  }
  if ( num <= 0 || probs.size() <= static_cast<size_t>(idx + num - 1) ) {
    return 0.;
  }
  float ret = 0.;
  for (auto i = idx; i < idx + num; ++i) {
    ret += probs[i];
  }
  return ret / num;
}

} // end of namespace whisper
#endif // WHISPER_UTIL__TRANSCRIPT_HPP_
