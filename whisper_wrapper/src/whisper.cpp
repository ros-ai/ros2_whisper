#include "whisper_wrapper/whisper.hpp"

namespace whisper {
Whisper::Whisper(const std::string &model_path) {
  params = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
  initialize(model_path);
}

Whisper::~Whisper() { whisper_free(ctx); }

void Whisper::initialize(const std::string &model_path) {
  ctx = whisper_init_from_file(model_path.c_str());
}

std::vector<std::string> Whisper::forward(const std::vector<float> &input, int n_processors) {
  whisper_full_parallel(ctx, params, input.data(), input.size(),
                        n_processors); // TODO: error handling????
  std::vector<std::string> segments;
  int n_segments = whisper_full_n_segments(ctx);
  for (int i = 0; i < n_segments; i++) {
    segments.push_back(whisper_full_get_segment_text(ctx, i));
  }
  return segments;
}

} // end of namespace whisper
