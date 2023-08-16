#ifndef ROS2_WHISPER__WHISPER_HPP_
#define ROS2_WHISPER__WHISPER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "whisper.h"

namespace ros2_whisper {
class Whisper {
public:
  Whisper();
  ~Whisper();

  std::vector<std::string> forward(const std::vector<float> &input);

protected:
  whisper_context *wctx_;
  whisper_full_params wparams_;
};
} // end of namespace ros2_whisper

#endif // ROS2_WHISPER__WHISPER_HPP_