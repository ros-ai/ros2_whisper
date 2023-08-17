#include "whisper_component.hpp"

namespace whisper {
WhisperComponent::WhisperComponent(const rclcpp::NodeOptions &options) : Node("whisper", options) {
  initialize_parameters_();

  // create inference service
  inference_service_ = this->create_service<whisper_msgs::srv::Inference>(
      "~/inference", std::bind(&WhisperComponent::on_inference_, this, std::placeholders::_1,
                               std::placeholders::_2));

  // create audio provide client
  provide_client_ = this->create_client<whisper_msgs::srv::ProvideAudio>("~/provide");
  int max_attempts = 5;
  int attempts = 0;
  while (!provide_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      throw std::runtime_error("Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for the service %s to become available %d/%d...",
                provide_client_->get_service_name(), attempts + 1, max_attempts);
    if (++attempts == max_attempts) {
      std::string err_msg = "Service " + std::string(provide_client_->get_service_name()) +
                            " is not available after " + std::to_string(max_attempts) +
                            " attempts.";
      RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
      throw std::runtime_error(err_msg);
    }
  }
  RCLCPP_INFO(this->get_logger(), "Service %s is now available.",
              provide_client_->get_service_name());

  // initialize model
  std::string model_name = this->get_parameter("model_name").as_string();
  RCLCPP_INFO(this->get_logger(), "Checking if model %s is available...", model_name.c_str());
  if (!model_manager_.is_available(model_name)) {
    RCLCPP_INFO(this->get_logger(), "Model %s is not available. Attempting download...",
                model_name.c_str());
    if (model_manager_.make_available(model_name) != 0) {
      std::string err_msg = "Failed to download model " + model_name + ".";
      RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
      throw std::runtime_error(err_msg);
    }
    RCLCPP_INFO(this->get_logger(), "Model %s downloaded.", model_name.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Model %s is available.", model_name.c_str());

  RCLCPP_INFO(this->get_logger(), "Initializing model %s...", model_name.c_str());
  whisper_.initialize(model_manager_.get_model_path(model_name));
  RCLCPP_INFO(this->get_logger(), "Model %s initialized.", model_name.c_str());
}

void WhisperComponent::initialize_parameters_() {
  if (!this->has_parameter("model_name")) {
    this->declare_parameter("model_name", rclcpp::ParameterValue("base.en"));
  }
  if (!this->has_parameter("language")) {
    this->declare_parameter("language", rclcpp::ParameterValue("en"));
  }
  if (!this->has_parameter("n_threads")) {
    this->declare_parameter("n_threads", rclcpp::ParameterValue(1));
  }
}

void WhisperComponent::on_inference_(const whisper_msgs::srv::Inference::Request::SharedPtr request,
                                     whisper_msgs::srv::Inference::Response::SharedPtr response) {
  // call provide client
  

  // run inference
}

} // end of namespace whisper

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(whisper::WhisperComponent)
