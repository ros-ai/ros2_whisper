#include "ros2_whisper/inference_server.hpp"

namespace ros2_whisper {
InferenceServer::InferenceServer(const rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_(node_ptr), record_audio_(false),
      whisper_(node_ptr->get_node_parameters_interface()) {

  record_service_ = node_ptr_->create_service<std_srvs::srv::SetBool>(
      "~/record", [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                         std_srvs::srv::SetBool::Response::SharedPtr response) {
        record_audio_ = request->data;
        response->message = record_audio_ ? "Recording audio." : "Stopped recording audio.";
        response->success = true;
      });

  clear_service_ = node_ptr_->create_service<std_srvs::srv::Trigger>(
      "~/clear", [this](const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
                        std_srvs::srv::Trigger::Response::SharedPtr response) {
        audio_buffer_.clear_audio_data();
        response->message = "Cleared audio buffer.";
        response->success = true;
      });

  inference_service_ = node_ptr_->create_service<ros2_whisper_msgs::srv::Inference>(
      "~/inference", std::bind(&InferenceServer::on_inference, this, std::placeholders::_1,
                               std::placeholders::_2));

  audio_subscription_ = node_ptr_->create_subscription<std_msgs::msg::Int16MultiArray>(
      "~/audio", 10,
      [this](std_msgs::msg::Int16MultiArray::SharedPtr msg) { // TODO: HACKED topic name!!
        if (record_audio_) {
          audio_buffer_.add_audio_data(msg->data);
          RCLCPP_INFO(node_ptr_->get_logger(), "Audio buffer length: %ld",
                      audio_buffer_.get_audio_data_size());
        }
      });
}

void InferenceServer::on_inference(
    const ros2_whisper_msgs::srv::Inference::Request::SharedPtr requset,
    ros2_whisper_msgs::srv::Inference::Response::SharedPtr response) {
  if (audio_buffer_.get_audio_data_size() == 0) {
    response->message = "Audio buffer is empty.";
    RCLCPP_WARN(node_ptr_->get_logger(), response->message.c_str());
    response->success = false;
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "Running inference...");
  auto segments =
      whisper_.forward(audio_buffer_.get_normalized_audio_data(), requset->n_processors);
  RCLCPP_INFO(node_ptr_->get_logger(), "Done.");
  response->message = "Inference successful.";
  response->text = segments;
  response->success = true;
}

} // end of namespace ros2_whisper
