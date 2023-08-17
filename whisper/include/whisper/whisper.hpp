#ifndef WHISPER__WHISPER_HPP_
#define WHISPER__WHISPER_HPP_

#include <string>
#include <vector>

#include "whisper.h"

namespace whisper {
class Whisper {
public:
  Whisper() = default;
  Whisper(const std::string &model_path);
  ~Whisper();

  void initialize(const std::string &model_path);
  std::vector<std::string> forward(const std::vector<float> &input, int n_processors = 1);

  whisper_context *ctx;
  whisper_full_params params;
};
} // end of namespace whisper

#endif // WHISPER__WHISPER_HPP_
