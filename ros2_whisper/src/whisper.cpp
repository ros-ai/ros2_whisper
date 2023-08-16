#include "ros2_whisper/whisper.hpp"

namespace ros2_whisper {
Whisper::Whisper() {
  wparams_ = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
  std::string model_path = "/opt/whisper/ggml-base.en.bin";
  wctx_ = whisper_init_from_file(model_path.c_str());
}

Whisper::~Whisper() { whisper_free(wctx_); }

std::vector<std::string> Whisper::forward(const std::vector<float> &input) {
  whisper_full_parallel(wctx_, wparams_, input.data(), input.size(), 1);
  std::vector<std::string> segments;
  int n_segments = whisper_full_n_segments(wctx_);
  for (int i = 0; i < n_segments; i++) {
    segments.push_back(whisper_full_get_segment_text(wctx_, i));
  }
  return segments;
}

} // end of namespace ros2_whisper
