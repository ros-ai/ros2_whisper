#ifndef WHISPER_UTIL__AUDIO_BUFFERS_HPP_
#define WHISPER_UTIL__AUDIO_BUFFERS_HPP_

#include <chrono>
#include <limits>
#include <mutex>
#include <vector>

#include "whisper.h"

namespace whisper {
/**
 * @brief A ring buffer implementation. This buffer is **not** thread-safe. It is the user's
 * responsibility to ensure thread-safety.
 *
 * @tparam value_type
 */
template <typename value_type> class RingBuffer {
  using const_reference = const value_type &;

public:
  RingBuffer(const std::size_t &capacity);

  void enqueue(const_reference data);
  value_type dequeue();
  inline bool is_full() const { return size_ == capacity_; }
  void clear();

  inline const std::size_t &capacity() const { return capacity_; }
  inline const std::size_t &size() const { return size_; }

protected:
  void increment_head_();
  void increment_tail_();

  std::size_t capacity_;
  std::vector<value_type> buffer_;
  std::size_t head_;
  std::size_t tail_;
  std::size_t size_;
};

/**
 * @brief A thread-safe buffer for storing audio data. The user enqueues data from an audio stream
 * in thread A and dequeues data in thread B. When the maximum batch capacity is reached, the audio
 * data is cleared.
 *
 * Thread A: enqueue into audio_buffer_ (ring buffer)
 * Thread B: dequeue from audio_buffer_ and store into audio_ up to batch_capacity_
 *
 */
class BatchedBuffer {
public:
  BatchedBuffer(
      const std::chrono::milliseconds &batch_capacity = std::chrono::seconds(5),
      const std::chrono::milliseconds &audio_buffer_capacity = std::chrono::seconds(2),
      const std::chrono::milliseconds &carry_over_capacity = std::chrono::milliseconds(200));

  void enqueue(const std::vector<std::int16_t> &audio);
  std::vector<float> dequeue();
  void clear();

  inline const std::uint16_t &batch_idx() const { return batch_idx_; };

protected:
  inline std::size_t time_to_sample_count_(const std::chrono::milliseconds &ms) {
    return ms.count() * WHISPER_SAMPLE_RATE / 1e3;
  };

  bool is_new_batch_();
  void carry_over_();

  std::mutex mutex_;

  std::size_t batch_capacity_;
  std::size_t carry_over_capacity_;
  std::uint16_t batch_idx_;

  std::vector<float> audio_;
  RingBuffer<std::int16_t> audio_buffer_;
};
} // end of namespace whisper
#endif // WHISPER_UTIL__AUDIO_BUFFERS_HPP_
