#include "whisper_util/audio_buffers.hpp"

namespace whisper {
EpisodicBuffer::EpisodicBuffer(const std::chrono::milliseconds &max_capacity,
                               const std::chrono::milliseconds &carry_over)
    : max_capacity_(time_to_sample_count_(max_capacity)),
      carry_over_(time_to_sample_count_(carry_over)){

      };

void EpisodicBuffer::append_new_audio(const std::vector<std::int16_t> &audio) {
  // TODO: this must be a ring buffer!
  std::lock_guard<std::mutex> lock(mutex_);
  std::transform(audio.begin(), audio.end(), std::back_inserter(audio_new_),
                 [](const std::int16_t &sample) {
                   return static_cast<float>(sample) /
                          static_cast<float>(std::numeric_limits<std::int16_t>::max());
                 });
}

void EpisodicBuffer::append_audio_from_new() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (audio_new_.empty()) {
    return;
  }

  if (audio_.size() + audio_new_.size() > max_capacity_) {
    clear_audio_();
  }

  audio_.insert(audio_.end(), audio_new_.begin(), audio_new_.end());
  audio_new_.clear();
}

void EpisodicBuffer::clear_audio_() {
  // carry over old data
  std::vector<float> carry_over(audio_.end() - carry_over_, audio_.end());
  audio_.clear();
  audio_.insert(audio_.end(), carry_over.begin(), carry_over.end());
}

std::size_t EpisodicBuffer::time_to_sample_count_(const std::chrono::milliseconds &ms) {
  return ms.count() * WHISPER_SAMPLE_RATE / 1e3;
}
} // end of namespace whisper
