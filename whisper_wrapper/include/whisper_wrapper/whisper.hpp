#ifndef WHISPER_WRAPPER__WHISPER_HPP_
#define WHISPER_WRAPPER__WHISPER_HPP_

#include <numeric>
#include <string>
#include <vector>

#include "whisper.h"

namespace whisper {
class Whisper {
public:
  Whisper();
  Whisper(const std::string &model_path);
  ~Whisper();

  void initialize(const std::string &model_path);
  std::string forward(const std::vector<float> &input);
  std::vector<whisper_token> tokens();

  whisper_context *ctx;
  whisper_full_params params;
};
} // end of namespace whisper
#endif // WHISPER_WRAPPER__WHISPER_HPP_
