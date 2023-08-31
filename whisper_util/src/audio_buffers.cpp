#include "whisper_util/audio_buffers.hpp"

namespace whisper {
template <typename value_type>
RingBuffer<value_type>::RingBuffer(const std::size_t &capacity)
    : capacity_(capacity), buffer_(capacity) {
  clear();
};

template <typename value_type> void RingBuffer<value_type>::enqueue(const_reference data) {
  increment_head_();
  if (is_full()) {
    increment_tail_();
  }
  buffer_[head_] = data;
}

template <typename value_type> value_type RingBuffer<value_type>::dequeue() {
  increment_tail_();
  return buffer_[tail_];
}

template <typename value_type> void RingBuffer<value_type>::clear() {
  head_ = 0;
  tail_ = 0;
  size_ = 0;
}

template <typename value_type> void RingBuffer<value_type>::increment_head_() {
  ++head_;
  ++size_;
  if (head_ >= capacity_) {
    head_ = 0;
  }
}
template <typename value_type> void RingBuffer<value_type>::increment_tail_() {
  ++tail_;
  --size_;
  if (tail_ >= capacity_) {
    tail_ = 0;
  }
}

BatchedBuffer::BatchedBuffer(const std::chrono::milliseconds &batch_capacity,
                             const std::chrono::milliseconds &audio_buffer_capacity,
                             const std::chrono::milliseconds &carry_over_capacity)
    : batch_capacity_(time_to_sample_count_(batch_capacity)),
      carry_over_capacity_(time_to_sample_count_(carry_over_capacity)), batch_idx_(0),
      audio_buffer_(time_to_sample_count_(audio_buffer_capacity)){

      };

void BatchedBuffer::enqueue(const std::vector<std::int16_t> &audio) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto &data : audio) {
    audio_buffer_.enqueue(data);
  }
}

std::vector<float> BatchedBuffer::dequeue() {
  std::lock_guard<std::mutex> lock(mutex_);
  bool is_new_batch = is_new_batch_();
  if (is_new_batch) {
    ++batch_idx_;
    carry_over_();
  }
  audio_.reserve(audio_.size() + audio_buffer_.size());
  for (std::size_t i = 0; i < audio_buffer_.size(); ++i) {
    audio_.push_back(static_cast<float>(audio_buffer_.dequeue()) /
                     static_cast<float>(std::numeric_limits<std::int16_t>::max()));
  }
  return audio_;
}

bool BatchedBuffer::is_new_batch_() {
  return (audio_.size() + audio_buffer_.size()) > batch_capacity_;
}

void BatchedBuffer::carry_over_() {
  std::vector<float> carry_over(audio_.end() - carry_over_capacity_, audio_.end());
  audio_.clear();
  audio_.insert(audio_.end(), carry_over.begin(), carry_over.end());
}

void BatchedBuffer::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  audio_.clear();
  audio_buffer_.clear();
}
} // end of namespace whisper
