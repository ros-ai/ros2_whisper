#include "whisper_nodes/inference_node.hpp"

namespace whisper {
InferenceNode::InferenceNode(const rclcpp::Node::SharedPtr node_ptr) : node_ptr_(node_ptr) {
  initialize_parameters_();

  // create inference service
  inference_service_ = node_ptr_->create_service<whisper_msgs::srv::Inference>(
      "~/run",
      std::bind(&InferenceNode::on_inference_, this, std::placeholders::_1, std::placeholders::_2));

  // initialize model
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
}

void InferenceNode::initialize_parameters_() {
  if (!node_ptr_->has_parameter("model_name")) {
    node_ptr_->declare_parameter("model_name", rclcpp::ParameterValue("base.en"));
  }
  if (!node_ptr_->has_parameter("language")) {
    node_ptr_->declare_parameter("language", rclcpp::ParameterValue("en"));
  }
  if (!node_ptr_->has_parameter("n_threads")) {
    node_ptr_->declare_parameter("n_threads", rclcpp::ParameterValue(1));
  }
}

void InferenceNode::on_inference_(const whisper_msgs::srv::Inference::Request::SharedPtr request,
                                  whisper_msgs::srv::Inference::Response::SharedPtr response) {
  if (request->audio.data.size() <= 0) {
    response->info = "No audio data provided.";
    response->success = false;
    return;
  }
  response->segments = whisper_.forward(request->audio.data, request->n_processors);
  response->info = "Inference successful.";
  response->success = true;
}
} // end of namespace whisper
