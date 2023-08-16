#include "ros2_whisper/inference_server.hpp"

namespace ros2_whisper {
InferenceServer::InferenceServer(const rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_(node_ptr), record_audio_(false) {

  record_audio_service_ = node_ptr_->create_service<std_srvs::srv::SetBool>(
      "record_audio", [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                             std_srvs::srv::SetBool::Response::SharedPtr response) {
        record_audio_ = request->data;
        response->message = record_audio_ ? "Recording audio." : "Stopped recording audio.";
        response->success = true;
      });

  clear_audio_buffer_service_ = node_ptr_->create_service<std_srvs::srv::Trigger>(
      "clear_audio_buffer", [this](const std_srvs::srv::Trigger::Request::SharedPtr request,
                                   std_srvs::srv::Trigger::Response::SharedPtr response) {
        audio_buffer_.clear_audio_data();
        response->message = "Cleared audio buffer.";
        response->success = true;
      });

  run_inference_service_ = node_ptr_->create_service<std_srvs::srv::Trigger>(
      "run_inference", std::bind(&InferenceServer::on_inference, this, std::placeholders::_1,
                                 std::placeholders::_2));

  audio_subscription_ = node_ptr_->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/audio_listener_node/audio", 10,
      [this](std_msgs::msg::Int16MultiArray::SharedPtr msg) { // TODO: HACKED topic name!!
        if (record_audio_) {
          audio_buffer_.add_audio_data(msg->data);
          RCLCPP_INFO(node_ptr_->get_logger(), "Audio buffer length: %ld",
                      audio_buffer_.get_audio_data_size());
        }
      });
}

void InferenceServer::on_inference(
    const std_srvs::srv::Trigger::Request::SharedPtr
        requset, // TODO: to be replaced by custom service message, which includes inferenced text!
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  if (audio_buffer_.get_audio_data_size() == 0) {
    response->message = "Audio buffer is empty.";
    response->success = false;
  }
}

} // namespace ros2_whisper
