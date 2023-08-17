#include "whisper_nodes/whisper_node.hpp"

namespace whisper {
WhisperNode::WhisperNode(const rclcpp::Node::SharedPtr node_ptr) : node_ptr_(node_ptr) {
  initialize_parameters_();

  // create inference service
  inference_service_ = node_ptr_->create_service<whisper_msgs::srv::Inference>(
      "~/inference",
      std::bind(&WhisperNode::on_inference_, this, std::placeholders::_1, std::placeholders::_2));

  // create audio provide client
  provide_client_ = node_ptr_->create_client<whisper_msgs::srv::ProvideAudio>("~/provide");
  int max_attempts = 5;
  int attempts = 0;
  while (!provide_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      throw std::runtime_error("Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Waiting for the service %s to become available %d/%d...",
                provide_client_->get_service_name(), attempts + 1, max_attempts);
    if (++attempts == max_attempts) {
      std::string err_msg = "Service " + std::string(provide_client_->get_service_name()) +
                            " is not available after " + std::to_string(max_attempts) +
                            " attempts.";
      RCLCPP_ERROR(node_ptr_->get_logger(), err_msg.c_str());
      throw std::runtime_error(err_msg);
    }
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "Service %s is now available.",
              provide_client_->get_service_name());

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
  // call provide client
  auto provide_request = std::make_shared<whisper_msgs::srv::ProvideAudio::Request>();

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