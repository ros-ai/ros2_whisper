#include "ros2_whisper/whisper.hpp"

namespace ros2_whisper {
Whisper::Whisper() {
  wparams_ = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
  std::string model_path = "/opt/whisper/ggml-base.en.bin";
  wctx_ = whisper_init_from_file(model_path.c_str());
}

Whisper::~Whisper() { whisper_free(wctx_); }

std::string Whisper::forward() {
  whisper_full_parallel(wctx_, wparams_, nullptr, 0, 1);
  return "TODO: inferenced text!";
}

} // end of namespace ros2_whisper
