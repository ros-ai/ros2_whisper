#ifndef ROS2_WHISPER__AUDIO_BUFFER_HPP_
#define ROS2_WHISPER__AUDIO_BUFFER_HPP_

#include <cstdint>
#include <vector>

namespace ros2_whisper {
class AudioBuffer {
public:
  AudioBuffer();
  void add_audio_data(const std::vector<std::int16_t> &audio_data);
  void clear_audio_data();
  inline const std::vector<std::int16_t> &get_audio_data() const { return audio_data_; };
  inline std::size_t get_audio_data_size() const { return audio_data_.size(); };

protected:
  std::vector<std::int16_t> audio_data_;
};

} // end of namespace ros2_whisper

#endif // ROS2_WHISPER__AUDIO_BUFFER_HPP_
