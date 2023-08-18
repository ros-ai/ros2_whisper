#include "whisper_wrapper/audio_buffer.hpp"

namespace whisper {
AudioBuffer::AudioBuffer(std::size_t capacity) : ring_buffer(capacity) {}

std::vector<float> AudioBuffer::normalize(const Junk &junk) {
  std::vector<float> normalized_junk;
  std::for_each(junk.begin(), junk.end(), [&](const std::int16_t &value) {
    normalized_junk.push_back(static_cast<float>(value) / 32768.); // normalize to [-1, 1]
  });
  return normalized_junk;
}
} // end of namespace whisper
