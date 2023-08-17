#include "ros2_whisper/whisper.hpp"

namespace ros2_whisper {
Whisper::Whisper(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface)
    : parameter_interface_(parameter_interface) {
  initialize_parameters_();
  initialize_model_();
}

Whisper::~Whisper() { whisper_free(wctx_); }

std::vector<std::string> Whisper::forward(const std::vector<float> &input, int n_processors) {
  whisper_full_parallel(wctx_, wparams_, input.data(), input.size(), n_processors);
  std::vector<std::string> segments;
  int n_segments = whisper_full_n_segments(wctx_);
  for (int i = 0; i < n_segments; i++) {
    segments.push_back(whisper_full_get_segment_text(wctx_, i));
  }
  return segments;
}

void Whisper::initialize_parameters_() {
  if (!parameter_interface_->has_parameter("model_name")) {
    parameter_interface_->declare_parameter("model_name",
                                            rclcpp::ParameterValue("ggml-base.en.bin"));
  }
  if (!parameter_interface_->has_parameter("language")) {
    parameter_interface_->declare_parameter("language", rclcpp::ParameterValue("en"));
  }
  if (!parameter_interface_->has_parameter("n_threads")) {
    parameter_interface_->declare_parameter("n_threads", rclcpp::ParameterValue(1));
  }

  wparams_ = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
  wparams_.language = parameter_interface_->get_parameter("language").as_string().c_str();
  wparams_.n_threads = parameter_interface_->get_parameter("n_threads").as_int();
}

void Whisper::initialize_model_() {
  using namespace std;
  if (!model_manager_.is_available(parameter_interface_->get_parameter("model_name").as_string())) {
    try {
      model_manager_.make_available(parameter_interface_->get_parameter("model_name").as_string());
    } catch (const exception &e) {
      cerr << e.what() << '\n';
      cerr << "Failed to download model, please download manually and place in "
              "~/.cache/whisper.cpp"
           << endl;
      exit(1);
    }
  }
  wctx_ = whisper_init_from_file(model_manager_.get_model_path().c_str());
}
} // end of namespace ros2_whisper
