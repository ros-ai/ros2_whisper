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

EpisodicBuffer::EpisodicBuffer(const std::chrono::milliseconds &episode_capacity,
                               const std::chrono::milliseconds &audio_buffer_capacity,
                               const std::chrono::milliseconds &carry_over)
    : episode_capacity_(time_to_sample_count(episode_capacity)),
      carry_over_(time_to_sample_count(carry_over)),
      audio_buffer_(time_to_sample_count(audio_buffer_capacity)){

      };

void EpisodicBuffer::insert(const std::vector<std::int16_t> &audio) {
  std::lock_guard<std::mutex> lock(mutex_);
  audio_buffer_.insert(audio);
}

void EpisodicBuffer::append_audio_from_new() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (audio_buffer_.size() == 0) {
    // warn
  }

  if (audio_buffer_.size() > audio_buffer_.capacity()) {
    // warn
  }

  if (audio_.size() + audio_buffer_.size() > episode_capacity_) {
    // not clear how this should work. How to keep track of what data has been inferenced on?
    std::cout << "clearing audio buffer: "
              << " " << audio_.size() << " + " << audio_buffer_.size() << " < " << episode_capacity_
              << std::endl;
    clear_audio_();
  }

  auto audio_new = audio_buffer_.retrieve();
  audio_.insert(audio_.end(), audio_new.begin(), audio_new.end());
}

void EpisodicBuffer::clear_audio_() {
  // carry over old data
  std::vector<float> carry_over(audio_.end() - carry_over_, audio_.end());
  audio_.clear();
  audio_.insert(audio_.end(), carry_over.begin(), carry_over.end());
}
} // end of namespace whisper
