#ifndef WHISPER_UTIL__AUDIO_BUFFERS_HPP_
#define WHISPER_UTIL__AUDIO_BUFFERS_HPP_

#include <algorithm>
#include <chrono>
#include <limits>
#include <mutex>
#include <vector>

#include "whisper.h"

namespace whisper {
std::size_t time_to_sample_count(const std::chrono::milliseconds &ms);

class RingBuffer {
public:
  RingBuffer(const std::size_t &capacity);

  void insert(const std::vector<std::int16_t> &data);
  std::vector<float> retrieve();
  void reset();

  inline const std::size_t &capacity() const { return capacity_; };
  inline const std::size_t &size() const { return size_; };

protected:
  inline std::size_t increment_index_(const std::size_t &index) const {
    return (index + 1) % capacity_;
  }

  std::size_t capacity_;
  std::vector<std::int16_t> buffer_;
  std::mutex mutex_;
  std::size_t head_;
  std::size_t tail_;
  std::size_t size_;
};

class EpisodicBuffer {
public:
  EpisodicBuffer(const std::chrono::milliseconds &episode_capacity = std::chrono::seconds(10),
                 const std::chrono::milliseconds &audio_buffer_capacity = std::chrono::seconds(2),
                 const std::chrono::milliseconds &carry_over = std::chrono::milliseconds(200));
  void insert(const std::vector<std::int16_t> &audio);
  void append_audio_from_new();

  inline std::vector<float> get_audio() const { return audio_; }

protected:
  void clear_audio_();

  std::mutex mutex_;

  std::size_t episode_capacity_;
  std::size_t carry_over_;

  std::vector<float> audio_;
  RingBuffer audio_buffer_;
};
} // end of namespace whisper
#endif // WHISPER_UTIL__AUDIO_BUFFERS_HPP_
