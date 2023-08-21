#ifndef WHISPER_WRAPPER__AUDIO_BUFFERS_HPP_
#define WHISPER_WRAPPER__AUDIO_BUFFERS_HPP_

#include <algorithm>
#include <chrono>
#include <limits>
#include <mutex>
#include <vector>

#include "whisper.h"

namespace whisper {
class EpisodicBuffer {
public:
  EpisodicBuffer(const std::chrono::milliseconds max_capacity = std::chrono::seconds(10),
                 const std::chrono::milliseconds &carry_over = std::chrono::milliseconds(200));
  void append_new_audio(const std::vector<std::int16_t> &audio);
  void append_audio_from_new();

  inline std::vector<float> get_audio() const;

protected:
  void clear_audio_();

  std::size_t time_to_sample_count_(const std::chrono::milliseconds &ms);

  // dequeue mutex
  std::mutex mutex_;

  std::size_t max_capacity_;
  std::size_t carry_over_;

  std::vector<float> audio_, audio_new_;
};
} // end of namespace whisper
#endif // WHISPER_WRAPPER__AUDIO_BUFFERS_HPP_
