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


/**
 * @brief Deserialized metadata about the segment.
 */
class SegmentMetaData {
private:
  SingleToken end_token_;
  std::chrono::milliseconds duration_;
  std::chrono::system_clock::time_point segment_start_;

public:

  SegmentMetaData(const SingleToken& end_token, 
                  std::chrono::milliseconds duration,
                  std::chrono::system_clock::time_point segment_start)
        : end_token_(end_token), duration_(duration), segment_start_(segment_start) {};


  SingleToken get_end_token() const { return end_token_; };
  std::chrono::milliseconds get_duration() const { return duration_; };
  std::chrono::system_clock::time_point get_start() const { return segment_start_; };

  void set_end_token(const SingleToken end_token) { end_token_ = end_token; };
  void set_duration(const std::chrono::milliseconds duration) { duration_ = duration; };
  void set_start(const std::chrono::system_clock::time_point segment_start) 
                                                  { segment_start_ = segment_start; };

  std::string get_end_token_data() const { return end_token_.get_data(); };

  void overwrite(const SegmentMetaData &other) {
    end_token_ = other.get_end_token();
    duration_ = other.get_duration();
    segment_start_ = other.get_start();
  }

  std::string as_str() const {
    return "[" + timestamp_as_str(segment_start_) + "(" + 
                      std::to_string(duration_.count()) + " ms )]";
  }

  std::string as_timestamp_str() const {
    return "[" + timestamp_as_str(segment_start_) + "]";
  }
};


// Helper functions for

} // end of namespace whisper
#endif // TRANSCRIPT_MANAGER__TOKENS_AND_SEGMENTS_HPP_
