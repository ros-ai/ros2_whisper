#ifndef WHISPER_WRAPPER__AUDIO_BUFFER_HPP_
#define WHISPER_WRAPPER__AUDIO_BUFFER_HPP_

#include <algorithm>
#include <vector>

#include "rclcpp/experimental/buffers/ring_buffer_implementation.hpp"

namespace whisper {
class AudioBuffer {
  using Junk = std::vector<std::int16_t>;
  using RingBuffer = rclcpp::experimental::buffers::RingBufferImplementation<Junk>;

public:
  AudioBuffer(std::size_t capacity = 100);
  static std::vector<float> normalize(const Junk &junk);
  RingBuffer ring_buffer;
};
} // end of namespace whisper
#endif // WHISPER_WRAPPER__AUDIO_BUFFER_HPP_
