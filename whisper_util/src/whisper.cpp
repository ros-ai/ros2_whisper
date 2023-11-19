#include "whisper_util/whisper.hpp"

namespace whisper {
Whisper::Whisper() { wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY); }

Whisper::Whisper(const std::string &model_path) { initialize(model_path); }

Whisper::~Whisper() { whisper_free(ctx); }

void Whisper::initialize(const std::string &model_path) {
  ctx = whisper_init_from_file_with_params(model_path.c_str(), cparams);
}

std::string Whisper::forward(const std::vector<float> &input) {
  if (whisper_full(ctx, wparams, input.data(), input.size()) != 0) {
    return {};
  }
  std::vector<std::string> segments;
  int n_segments = whisper_full_n_segments(ctx);
  for (int i = 0; i < n_segments; ++i) {
    segments.push_back(whisper_full_get_segment_text(ctx, i));
  }
  return std::accumulate(segments.begin(), segments.end(), std::string());
}

std::vector<whisper_token> Whisper::tokens() {
  std::vector<whisper_token> tokens;
  const int n_segments = whisper_full_n_segments(ctx);
  for (int i = 0; i < n_segments; ++i) {
    const int token_count = whisper_full_n_tokens(ctx, i);
    for (int j = 0; j < token_count; ++j) {
      tokens.push_back(whisper_full_get_token_id(ctx, i, j));
    }
  }
  return tokens;
}
} // end of namespace whisper
