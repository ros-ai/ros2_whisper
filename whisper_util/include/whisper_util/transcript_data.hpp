#ifndef WHISPER_UTIL__TRANSCRIPT_DATA_HPP_
#define WHISPER_UTIL__TRANSCRIPT_DATA_HPP_

#include <vector>
#include <string>
#include <numeric> // accumulate
#include <stdexcept>
#include <cstdint>
#include <algorithm> // reverse
#include <tuple>


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
  bool is_punct_;
  bool is_segment_;

  // Calculated and cached
  std::vector<std::string> word_cache_;
  std::vector<std::string> comparable_word_cache_;

public:
  Word(std::vector<SingleToken> tokens) : is_punct_(false), is_segment_(false) {
  	if (tokens.empty()) {
  		throw std::runtime_error("Cannot initialze word vec with no tokens.");
  	} else {
      add(tokens);
    }
  };

  Word(SingleToken token, bool is_punct) : is_punct_(is_punct_), is_segment_(false) {
    add({token});
  };

  Word() : is_punct_(false), is_segment_(true) {
    add({{"", 0.0}});
  };

  size_t size() const {
    return word_tokens_.size();
  }

  bool is_segment() const {
    return is_segment_;
  }

  void inc_best() {
    word_occurances_[0]++;
  }

  int get_occurrences() const {
    return word_occurances_[0];
  }

  void clear() {
    word_tokens_.clear();
    word_occurances_.clear();
    is_punct_ = false;
    word_cache_.clear();
    comparable_word_cache_.clear();
  }

  std::string get_comparable() const {
    return comparable_word_cache_[0];
  }

  std::string get() const {
    return word_cache_[0];
  }

  std::string get(const int i) const {
    if (i >= size() || i < 0) {
      throw std::out_of_range("Index is out of range.");
    }
    return word_cache_[i];
  }

  // bool empty() const {
  //   return word_tokens.empty() || word_tokens[0].empty();
  // }

  void add(std::vector<SingleToken> new_word) {
    // TODO:  Insert in sorted set
    word_tokens_.push_back(new_word);
    word_occurances_.push_back(1);
    build_cache();
  }

  void build_cache() {
    std::string word;
    for (auto& token : word_tokens_[word_tokens_.size()-1]) {
      word += token.get_data();
    }
    word_cache_.push_back(word);
    comparable_word_cache_.push_back(compute_comparable(word));
  }

  std::string compute_comparable(const std::string word) {
    std::string temp = word;
    // std::string temp = remove_whitespace_(word);
    // to_lower_case_(temp);
    // replace_number_word_with_digit_(temp);
    return temp;
  }

  void swap(const int new_best) {
    if (new_best >= size() || new_best < 0) {
      throw std::out_of_range("Index is out of range.");
    }
    std::swap(word_tokens_[0], word_tokens_[new_best]);
    std::swap(word_occurances_[0], word_occurances_[new_best]);
    std::swap(word_cache_[0], word_cache_[new_best]);
    std::swap(comparable_word_cache_[0], comparable_word_cache_[new_best]);
  }

  std::pair<bool, int> get_match(const std::string &other_text) const {
    bool success = false;
    for (int i=0; i<word_tokens_.size(); i++) {
      if(word_cache_[i] == other_text) {
        return {true, i};
      }
    }
    return {false, -1};
  }

  void compare(const std::vector<SingleToken> &other,
                const std::string &other_text) {
    auto [match_found, match_idx] = get_match(other_text);
    if (match_found) {
      word_occurances_[match_idx]++;
    } else {
      add(other);
    }
  }

  std::vector<int> get_top_n_ids(const int min_count) const {
    std::vector<int> top_n;
    for (int i=0; i<word_tokens_.size(); i++) {
      if (word_occurances_[i] >= min_count) {
        top_n.push_back(i);
      }
    }
    return top_n;
  }

  std::string get_print_str(const int min_count) const {
    auto ids = get_top_n_ids(min_count);
    if (ids.size() <= 1) {
      return get();
    }
    std::string str = "_{";
    bool first_run = true;
    for (const auto id : ids) {
      if (!first_run) {
        str += " | ";
      }
      str += word_cache_[id];
      str += " (";
      str += word_occurances_[id];
      str += ")";
      first_run = false;
    }
    str += "}_";
    return str;
  }

  void print_all() const {
    printf("%s\n", get_print_str(-1).c_str());
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

inline std::pair<bool, int> join_tokens(const std::vector<std::string> &tokens, const int idx) {
  // Check if, starting from the idx, the tokens are a bracket which can be combined or removed
  //   Return:
  //      bool -- Tokens start with bracket (can be combined)
  //      int  -- Number of tokens to combine
  const std::vector<std::pair<std::string, std::string>> combine_brackets = {
      {"[", "]"}, {" [", "]"},
      {"{", "}"}, {" {", "}"},
      {"(", ")"}, {" (", ")"}
  };
  // Set a limit on how many tokens can be within brackets
  const size_t max_allowed_tokens_to_combine = 10;

  if (idx < 0 || tokens.size() <= static_cast<size_t>(idx)) {
    return {false, 0};
  }

  // Try to combine tokens
  for (auto &[start, end] : combine_brackets) { 
    if (tokens[idx].size() >= start.size() && tokens[idx].substr(0, start.size()) == start) {
      bool end_found = false;
      int end_idx = idx + 1;
      while (end_idx < tokens.size() && (end_idx-idx) <= max_allowed_tokens_to_combine) {
        if (tokens[end_idx].size() >= end.size() && tokens[end_idx].substr(0, end.size()) == end) {
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
