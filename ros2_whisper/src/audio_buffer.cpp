#include "ros2_whisper/audio_buffer.hpp"

namespace ros2_whisper {
AudioBuffer::AudioBuffer() {}

void AudioBuffer::add_audio_data(const std::vector<std::int16_t> &audio_data) {
  audio_data_.insert(audio_data_.end(), audio_data.begin(), audio_data.end());
}

void AudioBuffer::clear_audio_data() { audio_data_.clear(); }
} // namespace ros2_whisper
