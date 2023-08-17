#include "whisper_nodes/whisper_node.hpp"

namespace whisper {
WhisperNode::WhisperNode(const rclcpp::Node::SharedPtr node_ptr) : node_ptr_(node_ptr) {
  initialize_parameters_();

  // create inference service
  inference_service_ = node_ptr_->create_service<whisper_msgs::srv::Inference>(
      "~/inference",
      std::bind(&WhisperNode::on_inference_, this, std::placeholders::_1, std::placeholders::_2));

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

void WhisperNode::initialize_parameters_() {
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

void WhisperNode::on_inference_(const whisper_msgs::srv::Inference::Request::SharedPtr request,
                                whisper_msgs::srv::Inference::Response::SharedPtr response) {
  // retrieved audio
  // running inference
  // done

  // auto future_and_request_id = provide_client_->async_send_request(
  //     provide_request//,
  //     // [this](rclcpp::Client<whisper_msgs::srv::ProvideAudio>::SharedFuture future) {
  //     //   auto response = future.get();
  //     //   if (response->success) {
  //     //     // run inference
  //     //     auto segments = whisper_.forward(response->audio.data, 1);

  //     //     for (const auto &segment : segments) {
  //     //       RCLCPP_INFO(node_ptr_->get_logger(), "segment: %s", segment.c_str());
  //     //     }
  //     //   }
  //     // }
  //     );

  // future_and_request_id.future.get()->

  // if available run inference on cb?
}

} // end of namespace whisper
