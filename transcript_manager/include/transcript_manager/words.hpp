#ifndef TRANSCRIPT_MANAGER__WORDS_HPP_
#define TRANSCRIPT_MANAGER__WORDS_HPP_

#include <vector>
#include <string>
#include <utility>        // std::pair
#include <optional>       // std::optional
#include <stdexcept>      // std::runtime_error()

#include "transcript_manager/tokens.hpp"

namespace whisper {

/**
 * @brief A vector of tokens form a word.  
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
  std::vector<bool> word_is_punct_;
  std::vector<float> word_probs_;

  // Calculated and cached
  std::vector<std::string> word_cache_;
  std::vector<std::string> comparable_word_cache_;

public:
  Word(std::vector<SingleToken> tokens) {
    if ( tokens.empty() ) {
      throw std::runtime_error("Cannot initialze word vec with no tokens.");
    } else {
      add(tokens, false);
    }
  };

  Word(SingleToken token, bool is_punct) {
    add({token}, is_punct);
  };

  size_t size() const {
    return word_tokens_.size();
  }

  bool is_punct() const {
    return word_is_punct_[0];
  }

  void inc_best() {
    word_occurances_[0]++;
  }

  void dec_best() {
    word_occurances_[0]--;

    // Check for swap
    for (size_t i = 1; i < word_occurances_.size(); ++i) {
      if ( word_occurances_[0] < word_occurances_[i] ) {
        swap(i);
      }
    }
  }

  void clear() {
    word_tokens_.clear();
    word_occurances_.clear();
    word_is_punct_.clear();
    word_probs_.clear();
    word_cache_.clear();
    comparable_word_cache_.clear();
  }

  std::string get_comparable() const {
    if ( is_punct() || word_occurances_[0] <= 0 ) {
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

  inline void range_check(const int i, const size_t s) const {
    if ( i < 0 || i >= static_cast<int>(s) ) {
      throw std::out_of_range("Index is out of range.");
    }
  }

  std::string get(const int i) const {
    range_check(i, size());
    return word_cache_[i];
  }

  std::vector<SingleToken> get_best_tokens() const {
    return word_tokens_[0];
  }

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
    for (auto& token : word_tokens_[word_tokens_.size()-1]) {
      word += token.get_data();
      prob += token.get_prob();
    }
    prob /= word_tokens_[word_tokens_.size()-1].size();
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
    range_check(new_best, size());
    std::swap(word_tokens_[0], word_tokens_[new_best]);
    std::swap(word_occurances_[0], word_occurances_[new_best]);
    std::swap(word_cache_[0], word_cache_[new_best]);
    std::swap(word_probs_[0], word_probs_[new_best]);
    std::swap(word_is_punct_[0], word_is_punct_[new_best]);
    std::swap(comparable_word_cache_[0], comparable_word_cache_[new_best]);
  }

  std::pair<bool, int> get_match(const std::string &other_text) const {
    for (size_t i = 0; i < word_tokens_.size(); i++) {
      if ( word_cache_[i] == other_text ) {
        return {true, static_cast<int>(i)};
      }
    }
    return {false, -1};
  }

  void compare(const Word &no_conflict_other) {
    if ( auto [match_found, match_idx] = get_match(no_conflict_other.get()); match_found ) {
      // Average Probability
      word_probs_[match_idx] = (word_probs_[match_idx]*(word_occurances_[match_idx] - 1) +
                                  no_conflict_other.get_prob())  / word_occurances_[match_idx];

      word_occurances_[match_idx]++;
      if ( word_occurances_[match_idx] >= word_occurances_[0] ) {
        swap(match_idx);
      }
    } else {
      // Create new conflict
      add(no_conflict_other.get_best_tokens(), no_conflict_other.is_punct());
      if ( word_tokens_.size() > 1 && word_occurances_[0] <= 1 ) {
        swap(word_tokens_.size()-1);
      }
    }
  }

  std::vector<int> get_top_n_ids(const int min_count) const {
    std::vector<int> top_n;
    for (size_t i=0; i<word_tokens_.size(); i++) {
      if ( word_occurances_[i] >= min_count ) {
        top_n.push_back(i);
      }
    }
    return top_n;
  }

  std::string get_print_str(const int min_count) const {
    auto ids = get_top_n_ids(min_count);
    if ( ids.size() <= 1 ) {
      return get();
    }
    std::stringstream ss;
    ss << "{";
    bool first_run = true;
    for (const auto id : ids) {
      if ( !first_run ) {
        ss << "|";
      }
      ss << word_cache_[id];
      first_run = false;
    }
    ss <<  "}";
    return ss.str();
  }

  void print_all() const {
    printf("%s\n", get_print_str(-1).c_str());
  }

private:
  std::string to_lower_case_(const std::string &input) const {
    std::string result;
    for (char ch : input) {
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



} // end of namespace whisper
#endif // TRANSCRIPT_MANAGER__WORDS_HPP_