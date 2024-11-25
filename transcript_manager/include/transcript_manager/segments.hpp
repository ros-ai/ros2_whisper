#ifndef TRANSCRIPT_MANAGER__SEGMENTS_HPP_
#define TRANSCRIPT_MANAGER__SEGMENTS_HPP_

#include <chrono>
#include <vector>

#include "transcript_manager/words.hpp"

namespace whisper {

/**
 * @brief Deserialized metadata about the segment.
 */
class SegmentMetaData {
public:
  SingleToken end_token_;
  std::chrono::milliseconds duration_;
  std::chrono::system_clock::time_point start_;

  SegmentMetaData() : end_token_({"", 0.}) {};
  SegmentMetaData(const SingleToken& end_token, 
                  std::chrono::milliseconds duration,
                  std::chrono::system_clock::time_point segment_start)
        : end_token_(end_token), duration_(duration), start_(segment_start) {};

  SingleToken get_end_token() const { return end_token_; };
  std::chrono::milliseconds get_duration() const { return duration_; };
  std::chrono::system_clock::time_point get_start() const { return start_; };

  void set_end_token(const SingleToken end_token) { end_token_ = end_token; };
  void set_duration(const std::chrono::milliseconds duration) { duration_ = duration; };
  void set_start(const std::chrono::system_clock::time_point segment_start) 
                                                  { start_ = segment_start; };

  std::string get_end_token_data() const { return end_token_.get_data(); };

  void overwrite(const SegmentMetaData &other) {
    end_token_ = other.get_end_token();
    duration_ = other.get_duration();
    start_ = other.get_start();
  }

  std::string as_str() const {
    return "[" + timestamp_as_str(start_) + "(" + 
                      std::to_string(duration_.count()) + " ms )]";
  }

  std::string as_timestamp_str() const {
    return "[" + timestamp_as_str(start_) + "]";
  }
};

/**
 * @brief A segment stores words with associated metadata.  
 */
class Segment {
public:
  SegmentMetaData data_;
  std::vector<Word> words_;
  int occ;

  Segment() : occ(0) {};
  Segment(const std::vector<Word> words) : words_(words), occ(0) {};
  Segment(const SegmentMetaData data)
              : data_(data), occ(0) {};
  Segment(const std::vector<Word> words, const SegmentMetaData data)
              : data_(data), words_(words), occ(0) {};

  void set_duration_to(const Segment& next) {
    if (data_.start_ > next.data_.start_) {
      data_.duration_ = std::chrono::milliseconds(0);
    } else {
      data_.duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                   next.data_.start_ - data_.start_);
    }
  }

  void clear() {
    words_.clear();
    // data can be overwritten;
  }

  void set_duration(std::chrono::milliseconds duration) { data_.duration_ = duration; };
  std::chrono::milliseconds get_duration() const { return data_.duration_; };
  std::chrono::system_clock::time_point get_start() const { return data_.start_; };

  void overwrite(const Segment &other) {
    data_ = other.data_;
  }

  void inc() {
    occ++;
  }

  void dec() {
    occ--;
  }

  std::string get_words() const {
    std::string ret;
    for (const auto& word : words_) {
      ret += word.get();
    }
    return ret;
  }

  std::string as_str() const {
    return data_.as_str() + ":  " + get_words();
  }

  std::string as_timestamp_str() const {
    return data_.as_str();
  }

};

} // end of namespace whisper
#endif // TRANSCRIPT_MANAGER__SEGMENTS_HPP_