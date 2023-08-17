#include "whisper_nodes/audio_buffer_node.hpp"

namespace whisper {
AudioBufferNode::AudioBufferNode(const rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_(node_ptr), record_(false) {

  audio_subscription_ = node_ptr_->create_subscription<std_msgs::msg::Int16MultiArray>(
      "~/audio", 10, [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (record_) {
          audio_buffer_.add_audio_data(msg->data);
        }
      });

  record_audio_action_server_ = rclcpp_action::create_server<RecordAudioAction>(
      node_ptr_, "~/record",
      std::bind(&AudioBufferNode::on_record_audio_goal_, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&AudioBufferNode::on_record_audio_cancel_, this, std::placeholders::_1),
      std::bind(&AudioBufferNode::on_record_audio_accepted_, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse
AudioBufferNode::on_record_audio_goal_(const rclcpp_action::GoalUUID &uuid,
                                       const RecordAudioAction::Goal::ConstSharedPtr goal) {}

rclcpp_action::CancelResponse
AudioBufferNode::on_record_audio_cancel_(const std::shared_ptr<RecordAudioGoalHandle> goal_handle) {
}

void AudioBufferNode::on_record_audio_accepted_(
    const std::shared_ptr<RecordAudioGoalHandle> goal_handle) {}

void AudioBufferNode::record_audio_(const std::shared_ptr<RecordAudioGoalHandle> goal_handle) {}
} // end of namespace whisper
