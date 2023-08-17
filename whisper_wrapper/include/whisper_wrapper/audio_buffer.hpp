#ifndef WHISPER_WRAPPER__AUDIO_BUFFER_HPP_
#define WHISPER_WRAPPER__AUDIO_BUFFER_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace whisper {
class AudioBuffer {
public:
  AudioBuffer();
  void add_audio_data(const std::vector<std::int16_t> &audio_data);
  void clear_audio_data();
  inline const std::vector<std::int16_t> &get_audio_data() const { return audio_data_; };
  std::vector<float> get_normalized_audio_data() const;
  inline std::size_t get_audio_data_size() const { return audio_data_.size(); };

protected:
  std::vector<std::int16_t> audio_data_;
};
} // end of namespace whisper
#endif // WHISPER_WRAPPER__AUDIO_BUFFER_HPP_
