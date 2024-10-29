#ifndef WHISPER_UTIL__TRANSCRIPT_DATA_HPP_
#define WHISPER_UTIL__TRANSCRIPT_DATA_HPP_

#include <vector>
#include <string>
#include <numeric> // accumulate
#include <stdexcept>
#include <cstdint>
#include <algorithm> // reverse
#include <tuple>
#include <iomanip>
#include <sstream>

namespace whisper {


// Declare Helper Functions
inline bool is_special_token(const std::vector<std::string> &tokens, const int idx);
inline bool my_ispunct(const std::vector<std::string> &tokens, const int idx);
inline std::pair<bool, int> join_tokens(const std::vector<std::string> &tokens, const int idx);
inline std::string combine_text(const std::vector<std::string> &tokens, 
                                                      const int idx, const int num);
inline float combine_prob(const std::vector<float> &probs, const int idx, const int num);


/**
 * @brief Deserialized data of the token from the WhisperTokens.msg
 */
class SingleToken {
private:
  std::string data_;
  float prob_;
  int token_id_;

public:
  std::string get_data() const {
    return data_;
  }

  float get_prob() const {
    return prob_;
  }

  SingleToken(const std::string& data_, float prob_)
        : data_(data_), prob_(prob_) {};

  // Copy constructor
  SingleToken(const SingleToken& other)
      : data_(other.data_), prob_(other.prob_), token_id_(other.token_id_) {};

  // Move constructor
  SingleToken(SingleToken&& other) noexcept
      : data_(std::move(other.data_)), prob_(other.prob_), token_id_(other.token_id_) {};

  // Copy assignment operator
  SingleToken& operator=(const SingleToken& other) {
    if (this != &other) {
      data_ = other.data_;
      prob_ = other.prob_;
      token_id_ = other.token_id_;
    }
    return *this;
  }

  // Move assignment operator
  SingleToken& operator=(SingleToken&& other) noexcept {
    if (this != &other) {
      data_ = std::move(other.data_);
      prob_ = other.prob_;
      token_id_ = other.token_id_;
    }
    return *this;
  }
};


/**
 * @brief Store metadata about the segment.
 */
class SegmentMetaData {
// private:
public:
  int word_array_start_idx_;
  int len_;
  SingleToken end_token_;
  int64_t start_time_;
  int64_t end_time_;

  SegmentMetaData(int word_array_start_idx, int len, SingleToken end_token,
                  int64_t start_time, int64_t end_time)
      : word_array_start_idx_(word_array_start_idx), len_(len), end_token_(end_token),
        start_time_(start_time), end_time_(end_time) {};

  SegmentMetaData()
      : end_token_({"", 0.}) {};
  
  std::string get_data() {
    return end_token_.get_data();
  }
};



/**
 * @brief A vector of tokens that form a word.  
 * Keep *N* different choices for the word (token array) in a sorted set.
 * 
 * token_vec_[0] -> Best word choice
 * token_vec_[1] -> Second best word choice
 * token_vec_[n] -> ...
 *     --- This behavior is not currently implmented and the vector is only best-value-first.
 */
class Word {
private:
  std::vector<std::vector<SingleToken>> word_tokens_;
  std::vector<int> word_occurances_;
  // SegmentMetaData segment_data_;
  std::vector<bool> word_is_punct_;
  std::vector<float> word_probs_;
  bool is_segment_;

  // Calculated and cached
  std::vector<std::string> word_cache_;
  std::vector<std::string> comparable_word_cache_;

public:
  Word(std::vector<SingleToken> tokens) : is_segment_(false) {
  	if (tokens.empty()) {
  		throw std::runtime_error("Cannot initialze word vec with no tokens.");
  	} else {
      add(tokens, false);
    }
  };

  Word(SingleToken token, bool is_punct) : is_segment_(false) {
    add({token}, is_punct);
  };

  Word() : is_segment_(true) {
    add({{"", 0.0}}, false);
  };

  size_t size() const {
    return word_tokens_.size();
  }

  bool is_segment() const {
    return is_segment_;
  }

  bool is_punct() const {
    return word_is_punct_[0];
  }

  void inc_best() {
    word_occurances_[0]++;
  }

  void dec_best() {
    word_occurances_[0]--;
    // Check for swaps
    for (size_t i = 1; i < word_occurances_.size(); ++i) {
      if (word_occurances_[0] < word_occurances_[i]) {
        swap(i);
      }
    }
  }

  void clear() {
    word_tokens_.clear();
    word_occurances_.clear();
    word_is_punct_.clear();
    word_cache_.clear();
    word_probs_.clear();
    comparable_word_cache_.clear();
  }

  std::string get_comparable() const {
    if (is_punct() || is_segment_ || word_occurances_[0] <= 0) {
      return "";
    }
    return comparable_word_cache_[0];
  }

  std::string get() const {
    return word_cache_[0];
  }
  
  int get_occurrences() const {
    return word_occurances_[0];
  }

  float get_prob() const {
    return word_probs_[0];
  }

  std::string get(const int i) const {
    if (i >= static_cast<int>(size()) || i < 0) {
      throw std::out_of_range("Index is out of range.");
    }
    return word_cache_[i];
  }

  std::vector<SingleToken> get_best_tokens() const {
    return word_tokens_[0];
  }

  // bool empty() const {
  //   return word_tokens.empty() || word_tokens[0].empty();
  // }

  // void add(std::vector<SingleToken> new_word) {
  //   // TODO:  Insert in sorted set
  //   word_tokens_.push_back(new_word);
  //   word_occurances_.push_back(1);
  //   word_is_punct_.push_back(false);
  //   build_cache();
  // }

  void add(std::vector<SingleToken> new_word, bool is_punct) {
    // TODO:  Insert in sorted set
    word_tokens_.push_back(new_word);
    word_occurances_.push_back(1);
    word_is_punct_.push_back(is_punct);
    build_cache();
  }

  void build_cache() {
    std::string word;
    float prob = 0.;
    int prob_count = 0;
    for (auto& token : word_tokens_[word_tokens_.size()-1]) {
      word += token.get_data();
      prob += token.get_prob();
      prob_count++;
    }
    prob /= prob_count;
    word_cache_.push_back(word);
    word_probs_.push_back(prob);
    comparable_word_cache_.push_back(compute_comparable(word));
  }

  std::string compute_comparable(const std::string word) {
    std::string temp = to_lower_case_(word);
    remove_whitespace_(temp);
    // replace_number_word_with_digit_(temp);
    return temp;
  }

  void swap(const int new_best) {
    if (new_best >= static_cast<int>(size()) || new_best < 0) {
      throw std::out_of_range("Index is out of range.");
    }
    std::swap(word_tokens_[0], word_tokens_[new_best]);
    std::swap(word_occurances_[0], word_occurances_[new_best]);
    std::swap(word_cache_[0], word_cache_[new_best]);
    std::swap(word_probs_[0], word_probs_[new_best]);
    std::swap(word_is_punct_[0], word_is_punct_[new_best]);
    std::swap(comparable_word_cache_[0], comparable_word_cache_[new_best]);
  }

  std::pair<bool, int> get_match(const std::string &other_text) const {
    for (size_t i = 0; i < word_tokens_.size(); i++) {
      if (word_cache_[i] == other_text) {
        return {true, static_cast<int>(i)};
      }
    }
    return {false, -1};
  }

  // void compare(const std::vector<SingleToken> &other,
  //               const std::string &other_text) {
  //   auto [match_found, match_idx] = get_match(other_text);
  //   if (match_found) {
  //     word_occurances_[match_idx]++;
  //     if (word_occurances_[match_idx] > word_occurances_[0]) {
  //       swap(match_idx);
  //     }
  //   } else {
  //     add(other);
  //   }
  // }

  void compare(const Word &no_conflict_other) {
    // printf("\nFIXED CONFLICT Between: '%s' and '%s'.   ", get().c_str(), no_conflict_other.get().c_str());
    auto [match_found, match_idx] = get_match(no_conflict_other.get());
    if (match_found) {
      // Running average of the probability
      // printf("Match idx:  %d (occ:  %d),  Prob %.2f update to:  ", match_idx, word_occurances_[match_idx], word_probs_[match_idx]);
      word_occurances_[match_idx]++;
      word_probs_[match_idx] = (word_probs_[match_idx]*(word_occurances_[match_idx] - 1) +
                                  no_conflict_other.get_prob())  / word_occurances_[match_idx];
      // printf("%.2f.\n", word_probs_[match_idx]);
      if (word_occurances_[match_idx] >= word_occurances_[0]) {
        swap(match_idx);
      }
    } else {
      // printf("No match found... Adding as new word\n");
      add(no_conflict_other.get_best_tokens(), no_conflict_other.is_punct());
      // Re-run compare(..), consider swapping
      auto [match_found_new, match_idx_new] = get_match(no_conflict_other.get());
      if (match_found_new) {
        if (word_occurances_[match_idx_new] >= word_occurances_[0]) {
          swap(match_idx_new);
        }
      }
    }
  }

  std::vector<int> get_top_n_ids(const int min_count) const {
    std::vector<int> top_n;
    for (size_t i=0; i<word_tokens_.size(); i++) {
      if (word_occurances_[i] >= min_count) {
        top_n.push_back(i);
      }
    }
    return top_n;
  }

  // std::string get_print_str(const int min_count) const {
  //   auto ids = get_top_n_ids(min_count);
  //   if (ids.size() <= 1) {
  //     return get();
  //   }
  //   std::string str = "_{";
  //   bool first_run = true;
  //   for (const auto id : ids) {
  //     if (!first_run) {
  //       str += " | ";
  //     }
  //     str += word_cache_[id];
  //     str += " (";
  //     str += std::to_string(word_occurances_[id]);
  //     str += ")";
  //     first_run = false;
  //   }
  //   str += "}_";
  //   return str;
  // }
  std::string get_print_str(const int min_count) const {
    auto ids = get_top_n_ids(min_count);
    if (ids.size() <= 1) {
      return get();
    }
    std::stringstream ss;
    ss << "_{";
    bool first_run = true;
    for (const auto id : ids) {
      if (!first_run) {
        ss << " | ";
      }
      ss << word_cache_[id];
      ss << " (";
      ss << std::setprecision(2) << word_probs_[id];
      ss << "\\";
      ss << word_occurances_[id];
      ss << ")";
      first_run = false;
    }
    ss <<  "}_";
    return ss.str();
  }

  void print_all() const {
    printf("%s\n", get_print_str(-1).c_str());
  }

private:
  std::string to_lower_case_(const std::string &input) const {
    std::string result;
    for (char ch : input) {
      // result += std::tolower(static_cast<unsigned char>(ch));
      result += std::tolower(ch);
    }
    return result;
  }

  void remove_whitespace_(std::string &input) const {
    input.erase(std::remove_if(input.begin(), input.end(), ::isspace), input.end());
  }

  void replace_number_word_with_digit_(std::string &input) const {
    // TODO
    const std::unordered_map<std::string, std::string> number_map = {
        {"zero", "0"}, {"one", "1"}, {"two", "2"}, {"three", "3"}, {"four", "4"},
        {"five", "5"}, {"six", "6"}, {"seven", "7"}, {"eight", "8"}, {"nine", "9"},
        {"ten", "10"}, {"eleven", "11"}, {"twelve", "12"}, {"thirteen", "13"},
        {"fourteen", "14"}, {"fifteen", "15"}, {"sixteen", "16"}, {"seventeen", "17"},
        {"eighteen", "18"}, {"nineteen", "19"}, {"twenty", "20"}
    };
    for (const auto& pair : number_map) {
      size_t pos = input.find(pair.first);
      if (pos != std::string::npos) {
        input.replace(pos, pair.first.size(), pair.second);
      }
    }
  }
};


/**
 * @brief A data maanger.  
 */
class Transcript {
  using WordsAndSegments = std::pair<std::vector<Word>, std::vector<SegmentMetaData>>;

private:
  std::vector<Word> transcript_;
  std::vector<SegmentMetaData> segments_;
  int stale_id_;
  int last_segment_id_;

public:
  enum OperationType { INCREMENT, DECREMENT, INSERT, CONFLICT, REMOVE, MATCHED_WORD};
  struct Operation {
    const OperationType op_type_;
    const int id_;
    const int other_id_;

    Operation(const OperationType op_type, const int id) : 
                    op_type_(op_type), id_(id), other_id_(-1) {
      if (op_type_ == INSERT || 
          op_type_ == CONFLICT ||
          op_type_ == MATCHED_WORD) {
        throw std::runtime_error("Missing argument on Operation initialization.");
      }
    }
    Operation(const OperationType op_type, const int id, const int other_id) : 
                  op_type_(op_type), id_(id), other_id_(other_id) {}
  };
  using Operations = std::vector<Operation>;

  void push_back(const WordsAndSegments &words_and_segments);

  // Standard Operations
  // void run_op(const Operation operation, const WordsAndSegments &words_other);
  // run all operations
  void run(const Operations &operations, const WordsAndSegments &words_other);
  // run subset of operations
  void run(const Operations &operations);
  void insert_word(const int id,
                    const WordsAndSegments &new_words,
                    const int new_word_id);
  void inc_word(const int id);
  void dec_word(const int id);
  void remove_word(const int id);
  // void compare_inc_word(const int id,
  //                     const std::vector<WordsAndSegments> &others,
  //                     const int other_id);
  void conflict_merge_word(const int id,
                      const WordsAndSegments &others,
                      const int other_id);

  Transcript(): stale_id_(0), last_segment_id_(-1) {};

  bool empty() const {
    return transcript_.empty();
  }

  std::vector<Word> get_words() {
    return transcript_;
  }

  std::vector<Word> get_words_splice() {
    // TODO:  return itterator
    return std::vector<Word>(transcript_.begin() + stale_id_, transcript_.end());
  }

  std::string get_active_word(const int id) {
    return transcript_[id + stale_id_].get();
  }

  int get_stale_word_id() const {
    return stale_id_;
  }

  void set_stale_word_id(int stale_id) {
    stale_id_ = stale_id;
  }

  void clear_mistakes(int occurrence_threshold) {
    Transcript::Operations pending_ops;
    for (int id = stale_id_; id < static_cast<int>(transcript_.size()); ++id) {
      if (transcript_[id].get_occurrences() <= occurrence_threshold) {
        // printf("Removed:  %s (%d)\n", transcript_[id].get().c_str(),
        //                               transcript_[id].get_occurrences());
        pending_ops.push_back({REMOVE, id});
      }
    }
    run(pending_ops);
  }
};





// 
// Helper Functions
//
inline bool is_special_token(const std::vector<std::string> &tokens, const int idx) {
  const std::vector<std::string> special_token_start_strs = {
      "[_BEG_]", "[_TT_", " [_BEG_]", " [_TT_"
  };
  if (idx < 0 || tokens.size() <= static_cast<size_t>(idx)) {
    return false;
  }
  for (const auto &start : special_token_start_strs) { 
    if (tokens[idx].size() >= start.size() && tokens[idx].substr(0, start.size()) == start) {
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
  if (idx < 0 || tokens.size() <= static_cast<size_t>(idx)) {
    return false;
  }
  return std::find(punctuations.begin(), punctuations.end(), tokens[idx]) != punctuations.end();
}

inline bool contains_char(const std::string &str, const char target) {
  for (const auto &c : str) {
    if (target == c) {
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

  if (idx < 0 || tokens.size() <= static_cast<size_t>(idx)) {
    return {false, 0};
  }

  // Try to combine tokens
  for (auto &[start, end] : combine_brackets) { 
    if (contains_char(tokens[idx], start)) {
      int end_idx = idx + 1;
      while (end_idx < static_cast<int>(tokens.size()) && (end_idx-idx) 
                                            <= max_allowed_tokens_to_combine) {
        if (contains_char(tokens[end_idx], end)) {
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


// inline std::pair<bool, int> join_tokens(const std::vector<std::string> &tokens, const int idx) {
//   // Check if, starting from the idx, the tokens are a bracket which can be combined or removed
//   //   Return:
//   //      bool -- Tokens start with bracket (can be combined)
//   //      int  -- Number of tokens to combine
//   const std::vector<std::pair<std::string, std::string>> combine_brackets = {
//       {"[", "]"}, {" [", "]"},
//       {"{", "}"}, {" {", "}"},
//       {"(", ")"}, {" (", ")"}
//   };
//   // Set a limit on how many tokens can be within brackets
//   const int max_allowed_tokens_to_combine = 10;

//   if (idx < 0 || tokens.size() <= static_cast<size_t>(idx)) {
//     return {false, 0};
//   }

//   // Try to combine tokens
//   for (auto &[start, end] : combine_brackets) { 
//     if (tokens[idx].size() >= start.size() && tokens[idx].substr(0, start.size()) == start) {
//       int end_idx = idx + 1;
//       while (end_idx < static_cast<int>(tokens.size()) && (end_idx-idx) 
//                                             <= max_allowed_tokens_to_combine) {
//         if (tokens[end_idx].size() >= end.size() && tokens[end_idx].substr(0, end.size()) == end) {
//           return {true, end_idx - idx + 1};
//         }
//         end_idx++;
//       }
//       // We found the start but not the end, "combine" current token with itself
//       return {true, 1};
//     }
//   }
//   return {false, 0};
// }

inline std::string combine_text(const std::vector<std::string> &tokens, 
                                                      const int idx, const int num) {
  if (idx < 0 || tokens.size() <= static_cast<size_t>(idx)) {
    return "";
  }
  if (num <= 0 || tokens.size() <= static_cast<size_t>(idx + num - 1)) {
    return "";
  }
  std::string ret = "";
  for (auto i = idx; i < idx + num; ++i) {
    ret += tokens[i];
  }
  return ret;
}

inline float combine_prob(const std::vector<float> &probs, const int idx, const int num) {
  if (idx < 0 || probs.size() <= static_cast<size_t>(idx)) {
    return 0.;
  }
  if (num <= 0 || probs.size() <= static_cast<size_t>(idx + num - 1)) {
    return 0.;
  }
  float ret = 0.;
  for (auto i = idx; i < idx + num; ++i) {
    ret += probs[i];
  }
  return ret;
}











} // end of namespace whisper
#endif // WHISPER_UTIL__TRANSCRIPT_DATA_HPP_
