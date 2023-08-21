#include "whisper_nodes/inference_node.hpp"

namespace whisper {
InferenceNode::InferenceNode(const rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_(node_ptr), language_("en") {
  declare_parameters_();

  // audio subscription
  audio_sub_ = node_ptr_->create_subscription<std_msgs::msg::Int16MultiArray>(
      "audio", 10, std::bind(&InferenceNode::on_audio_, this, std::placeholders::_1));

  // inference action server
  inference_action_server_ = rclcpp_action::create_server<Inference>(
      node_ptr_, "listen",
      std::bind(&InferenceNode::on_listen_, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&InferenceNode::on_cancel_listen_, this, std::placeholders::_1),
      std::bind(&InferenceNode::on_listen_accepted_, this, std::placeholders::_1));

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

void InferenceNode::on_audio_(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
  episodic_buffer_.append_new_audio(msg->data);
}

rclcpp_action::GoalResponse InferenceNode::on_listen_(const rclcpp_action::GoalUUID &uuid,
                                                      std::shared_ptr<const Inference::Goal> goal) {
}

rclcpp_action::CancelResponse
InferenceNode::on_cancel_listen_(const std::shared_ptr<GoalHandleInference> goal_handle) {}

void InferenceNode::on_listen_accepted_(const std::shared_ptr<GoalHandleInference> goal_handle) {}
} // end of namespace whisper
