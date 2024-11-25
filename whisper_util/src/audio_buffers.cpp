#include "whisper_util/audio_buffers.hpp"

namespace whisper {

AudioRing::AudioRing(const std::chrono::milliseconds &buffer_capacity,
                      std::chrono::system_clock::time_point cur_time)
                                : ThreadSafeRing<std::int16_t>(time_to_count(buffer_capacity)),
                                time_inc_(count_to_time_ns(1)), audio_start_set(true) {
  clear();
  set_start_timestamp(cur_time);
}

AudioRing::AudioRing(const std::chrono::milliseconds &buffer_capacity)
                                : ThreadSafeRing<std::int16_t>(time_to_count(buffer_capacity)),
                                time_inc_(count_to_time_ns(1)), audio_start_set(false) {
  clear();
}

std::tuple<std::vector<float>, std::chrono::system_clock::time_point> AudioRing::peak() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<float> result;
  result.reserve(this->size());
  for (std::size_t i = 0; i < this->size_; ++i) {
    result.push_back(static_cast<float>(this->buffer_[(this->tail_ + i) % this->capacity_]) /
                     static_cast<float>(std::numeric_limits<std::int16_t>::max()));
  }
  return {result, audio_start_};
}

void AudioRing::clear() {
  // First clear
  ThreadSafeRing<std::int16_t>::clear();

  // Then, enqueue 2 seconds of silence
  enqueue(std::vector<std::int16_t>(WHISPER_SAMPLE_RATE*2, 0));
  audio_start_set = false;
}

void AudioRing::increment_tail_() {
  audio_start_ += time_inc_;
  ThreadSafeRing::increment_tail_();
}

size_t AudioRing::decay(std::chrono::system_clock::time_point cur_time) {
  auto audio_end = audio_start_ + count_to_time(size_);
  if (audio_end > cur_time) {
    // Somehow the audio buffer goes past the current time, nothing to do
    return 0;
  }
  std::chrono::milliseconds diff_ms = 
          std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - audio_end);
  size_t zeros_to_add = time_to_count(diff_ms);
  enqueue(std::vector<std::int16_t>(zeros_to_add, 0));
  return zeros_to_add;
}

void AudioRing::set_start_timestamp(std::chrono::system_clock::time_point cur_time) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Since we are setting the start time of the buffer based on the current time,
  //    subtract what data is in the buffer against the current time.
  std::chrono::milliseconds elapsed = count_to_time(size_);
  if (elapsed > 
          std::chrono::duration_cast<std::chrono::milliseconds>(cur_time.time_since_epoch())) {
    // Shouldn't be possible, cur time should be seconds from 1970
    audio_start_ = cur_time;
    return;
  }
  audio_start_ = cur_time - elapsed;
  audio_start_set = true;
}

std::chrono::system_clock::time_point AudioRing::get_start_timestamp() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return audio_start_;
}


} // end of namespace whisper
