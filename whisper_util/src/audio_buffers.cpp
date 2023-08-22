#include "whisper_util/audio_buffers.hpp"

namespace whisper {
std::size_t time_to_sample_count(const std::chrono::milliseconds &ms) {
  return ms.count() * WHISPER_SAMPLE_RATE / 1e3;
}

RingBuffer::RingBuffer(const std::size_t &capacity) : capacity_(capacity), buffer_(capacity) {
  reset();
};

void RingBuffer::insert(const std::vector<std::int16_t> &data) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto &sample : data) {
    buffer_[head_] = sample;
    head_ = increment_index_(head_);
    ++size_;
  }
}

std::vector<float> RingBuffer::retrieve() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<float> data(size_);
  std::for_each(data.begin(), data.end(), [this](float &sample) {
    sample = static_cast<float>(buffer_[tail_]) /
             static_cast<float>(std::numeric_limits<std::int16_t>::max());
    tail_ = increment_index_(tail_);
  });
  size_ = 0;
  return data;
}

void RingBuffer::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  head_ = 0;
  tail_ = 0;
  size_ = 0;
}

EpisodicBuffer::EpisodicBuffer(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr &logging_interface,
    const std::chrono::milliseconds &episode_capacity,
    const std::chrono::milliseconds &audio_buffer_capacity,
    const std::chrono::milliseconds &carry_over)
    : logging_interface_(logging_interface),
      episode_capacity_(time_to_sample_count(episode_capacity)),
      carry_over_(time_to_sample_count(carry_over)),
      audio_buffer_(time_to_sample_count(audio_buffer_capacity)){

      };

void EpisodicBuffer::insert_from_stream(const std::vector<std::int16_t> &audio) {
  std::lock_guard<std::mutex> lock(mutex_);
  audio_buffer_.insert(audio);
}

std::vector<float> EpisodicBuffer::retrieve_buffered_audio() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (audio_buffer_.size() == 0) {
    RCLCPP_WARN(logging_interface_->get_logger(), "No audio in buffer.");
    return;
  }

  if (audio_.size() + audio_buffer_.size() > episode_capacity_) {
    finish_episode_();
  }

  auto audio_new = audio_buffer_.retrieve();
  audio_.insert(audio_.end(), audio_new.begin(), audio_new.end());
  return audio_;
}

void EpisodicBuffer::finish_episode_() {
  // carry over old data
  std::vector<float> carry_over(audio_.end() - carry_over_, audio_.end());
  audio_.clear();
  audio_.insert(audio_.end(), carry_over.begin(), carry_over.end());
}
} // end of namespace whisper
