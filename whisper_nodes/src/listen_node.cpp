#include "whisper_nodes/listen_node.hpp"

namespace whisper {
ListenNode::ListenNode(const rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_(node_ptr), recording_(false) {

  audio_subscription_ = node_ptr_->create_subscription<std_msgs::msg::Int16MultiArray>(
      "audio", 10, [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (recording_) {
          audio_buffer_.add_audio_data(msg->data);
        }
      });

  record_audio_action_server_ = rclcpp_action::create_server<ListenAction>(
      node_ptr_, "record",
      std::bind(&ListenNode::on_record_audio_goal_, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ListenNode::on_record_audio_cancel_, this, std::placeholders::_1),
      std::bind(&ListenNode::on_record_audio_accepted_, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse
ListenNode::on_record_audio_goal_(const rclcpp_action::GoalUUID &uuid,
                                  const ListenAction::Goal::ConstSharedPtr goal) {
  if (recording_) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  recording_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ListenNode::on_record_audio_cancel_(const std::shared_ptr<ListenGoalHandle> goal_handle) {
  recording_ = false;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ListenNode::on_record_audio_accepted_(const std::shared_ptr<ListenGoalHandle> goal_handle) {

  std::thread record_audio_thread(&ListenNode::record_audio_, this, goal_handle);
  record_audio_thread.detach();
}

void ListenNode::record_audio_(const std::shared_ptr<ListenGoalHandle> goal_handle) {
  auto feedback = std::make_shared<ListenAction::Feedback>();
  while (recording_) {
    feedback->size = audio_buffer_.get_audio_data_size();
    goal_handle->publish_feedback(feedback);
  }
}
} // end of namespace whisper
