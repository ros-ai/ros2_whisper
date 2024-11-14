#ifndef TRANSCRIPT_MANAGER__TOKENS_AND_SEGMENTS_HPP_
#define TRANSCRIPT_MANAGER__TOKENS_AND_SEGMENTS_HPP_

#include <chrono>
#include <string>
#include <stdexcept>

#include "whisper_util/chrono_utils.hpp"

namespace whisper {

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
    if ( this != &other ) {
      data_ = other.data_;
      prob_ = other.prob_;
      token_id_ = other.token_id_;
    }
    return *this;
  }

  // Move assignment operator
  SingleToken& operator=(SingleToken&& other) noexcept {
    if ( this != &other ) {
      data_ = std::move(other.data_);
      prob_ = other.prob_;
      token_id_ = other.token_id_;
    }
    return *this;
  }
};

} // end of namespace whisper
#endif // TRANSCRIPT_MANAGER__TOKENS_AND_SEGMENTS_HPP_
