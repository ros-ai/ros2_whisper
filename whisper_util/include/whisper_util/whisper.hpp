#ifndef WHISPER_UTIL__WHISPER_HPP_
#define WHISPER_UTIL__WHISPER_HPP_

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

  // Run whisper forword on input and serialize the result into WhisperTokens.msg fields
  void forward_serialize(
                  const std::vector<float> &input,
                  std::vector<int> &token_ids,
                  std::vector<std::string> &token_texts,
                  std::vector<float> &token_probs,
                  std::vector<int> &segment_start_token_idx,
                  std::vector<int64_t> &segment_start_timestamp,
                  std::vector<int64_t> &segment_end_timestamp);

  whisper_context *ctx;
  whisper_full_params wparams;
  whisper_context_params cparams;
};
} // end of namespace whisper
#endif // WHISPER_UTIL__WHISPER_HPP_
