
#include "whisper_nodes/audio_buffer_node.hpp"

namespace whisper {
AudioBufferNode::AudioBufferNode(const rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_(node_ptr), record_(false) {

  // rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_service_;
  record_service_ = node_ptr_->create_service<std_srvs::srv::SetBool>(
      "~/record", [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                         std_srvs::srv::SetBool::Response::SharedPtr response) {
        record_ = request->data;
        response->success = true;
        response->message = "Recording set to " + std::to_string(record_) + ".";
      });

  clear_service_ = node_ptr_->create_service<std_srvs::srv::Trigger>(
      "~/clear", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr response) {
        audio_buffer_.clear_audio_data();
        response->success = true;
        response->message = "Audio buffer cleared.";
      });

  // rclcpp::Service<whisper_msgs::srv::ProvideAudio> provide_service_;
  provide_service_ = node_ptr_->create_service<whisper_msgs::srv::ProvideAudio>(
      "~/provide", [this](const whisper_msgs::srv::ProvideAudio::Request::SharedPtr,
                          whisper_msgs::srv::ProvideAudio::Response::SharedPtr response) {
        if (audio_buffer_.get_audio_data_size() == 0) {
          response->success = false;
          response->info = "No audio data available.";
          RCLCPP_WARN(node_ptr_->get_logger(), response->info.c_str());
          return;
        }
        response->success = true;
        response->info = "Audio data provided.";
        std_msgs::msg::MultiArrayDimension dim;
        dim.label = "audio";
        dim.size = audio_buffer_.get_audio_data_size();
        dim.stride = 1;
        response->audio.layout.data_offset = 0;
        response->audio.layout.dim.push_back(dim);
        response->audio.data = audio_buffer_.get_normalized_audio_data();
      });

  audio_subscription_ = node_ptr_->create_subscription<std_msgs::msg::Int16MultiArray>(
      "~/audio", 10, [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (record_) {
          audio_buffer_.add_audio_data(msg->data);
        }
      });
}
} // end of namespace whisper
