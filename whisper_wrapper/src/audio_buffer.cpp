#include "whisper_wrapper/audio_buffer.hpp"

namespace whisper {
AudioBuffer::AudioBuffer() {}

void AudioBuffer::add_audio_data(const std::vector<std::int16_t> &audio_data) {
  audio_data_.insert(audio_data_.end(), audio_data.begin(), audio_data.end());
}

std::vector<float> AudioBuffer::get_normalized_audio_data() const {
  std::vector<float> normalized_audio_data;
  int i = 0;
  std::for_each(audio_data_.begin(), audio_data_.end(), [&](const std::int16_t &audio_data) {
    normalized_audio_data.push_back(static_cast<float>(audio_data) /
                                    32768.); // normalize to [-1, 1]
    ++i;
  });
  return normalized_audio_data;
}

void AudioBuffer::clear_audio_data() { audio_data_.clear(); }
} // end of namespace whisper
