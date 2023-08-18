#include "whisper_nodes/inference_node.hpp"

namespace whisper {
InferenceNode::InferenceNode(const rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_(node_ptr), language_("en") {
  declare_parameters_();

  // create feedback subscription
  listen_feedback_subscription_ =
      node_ptr_->create_subscription<whisper_msgs::action::Listen_FeedbackMessage>(
          "listen/_action/feedback", 10,
          std::bind(&InferenceNode::on_listen_feedback_, this, std::placeholders::_1));

  // create inference service
  inference_service_ = node_ptr_->create_service<whisper_msgs::srv::Inference>(
      "inference",
      std::bind(&InferenceNode::on_inference_, this, std::placeholders::_1, std::placeholders::_2));

  // parameter callback handle
  on_parameter_set_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&InferenceNode::on_parameter_set_, this, std::placeholders::_1));

  // initialize model
  initialize_whisper_();
}

void InferenceNode::declare_parameters_() {
  node_ptr_->declare_parameter("model_name", "base.en");
  // consider other parameters:
  // https://github.com/ggerganov/whisper.cpp/blob/a4bb2df36aeb4e6cfb0c1ca9fbcf749ef39cc852/whisper.h#L351
  node_ptr_->declare_parameter("language", "en");
  node_ptr_->declare_parameter("n_threads", 1);
  node_ptr_->declare_parameter("print_progress", false);
}

void InferenceNode::initialize_whisper_() {
  std::string model_name = node_ptr_->get_parameter("model_name").as_string();
  RCLCPP_INFO(node_ptr_->get_logger(), "Checking if model %s is available...", model_name.c_str());
  if (!model_manager_.is_available(model_name)) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Model %s is not available. Attempting download...",
                model_name.c_str());
    if (model_manager_.make_available(model_name) != 0) {
      std::string err_msg = "Failed to download model " + model_name + ".";
      RCLCPP_ERROR(node_ptr_->get_logger(), err_msg.c_str());
      throw std::runtime_error(err_msg);
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Model %s downloaded.", model_name.c_str());
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "Model %s is available.", model_name.c_str());

  RCLCPP_INFO(node_ptr_->get_logger(), "Initializing model %s...", model_name.c_str());
  whisper_.initialize(model_manager_.get_model_path(model_name));
  RCLCPP_INFO(node_ptr_->get_logger(), "Model %s initialized.", model_name.c_str());

  language_ = node_ptr_->get_parameter("language").as_string();
  whisper_.params.language = language_.c_str();
  whisper_.params.n_threads = node_ptr_->get_parameter("n_threads").as_int();
  whisper_.params.print_progress = node_ptr_->get_parameter("print_progress").as_bool();
}

void InferenceNode::on_listen_feedback_(
    const whisper_msgs::action::Listen_FeedbackMessage::SharedPtr msg) {
  RCLCPP_INFO(node_ptr_->get_logger(), "Received feedback.");
}

void InferenceNode::on_inference_(const whisper_msgs::srv::Inference::Request::SharedPtr request,
                                  whisper_msgs::srv::Inference::Response::SharedPtr response) {
  if (request->audio.data.size() <= 0) {
    response->info = "No audio data provided.";
    RCLCPP_INFO(node_ptr_->get_logger(), response->info.c_str());
    response->success = false;
    return;
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "Running inference...");
  auto segments = whisper_.forward(request->audio.data, request->n_processors);
  response->text = std::accumulate(segments.begin(), segments.end(), std::string());
  response->info = "Inference successful.";
  RCLCPP_INFO(node_ptr_->get_logger(), response->info.c_str());
  response->success = true;
}

rcl_interfaces::msg::SetParametersResult
InferenceNode::on_parameter_set_(const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto &parameter : parameters) {
    if (parameter.get_name() == "n_threads") {
      whisper_.params.n_threads = parameter.as_int();
      RCLCPP_INFO(node_ptr_->get_logger(), "Parameter %s set to %d.", parameter.get_name().c_str(),
                  whisper_.params.n_threads);
      continue;
    }
    result.reason = "Parameter " + parameter.get_name() + " not handled.";
    result.successful = false;
    RCLCPP_WARN(node_ptr_->get_logger(), result.reason.c_str());
  }
  result.successful = true;
  return result;
}
} // end of namespace whisper
