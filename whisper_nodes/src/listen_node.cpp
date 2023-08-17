#include "whisper_nodes/listen_node.hpp"

namespace whisper {
ListenNode::ListenNode(const rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_(node_ptr), record_(false) {

  audio_subscription_ = node_ptr_->create_subscription<std_msgs::msg::Int16MultiArray>(
      "~/audio", 10, [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (record_) {
          audio_buffer_.add_audio_data(msg->data);
        }
      });

  record_audio_action_server_ = rclcpp_action::create_server<ListenAction>(
      node_ptr_, "~/record",
      std::bind(&ListenNode::on_record_audio_goal_, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ListenNode::on_record_audio_cancel_, this, std::placeholders::_1),
      std::bind(&ListenNode::on_record_audio_accepted_, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse
ListenNode::on_record_audio_goal_(const rclcpp_action::GoalUUID &uuid,
                                  const ListenAction::Goal::ConstSharedPtr goal) {}

rclcpp_action::CancelResponse
ListenNode::on_record_audio_cancel_(const std::shared_ptr<ListenGoalHandle> goal_handle) {}

void ListenNode::on_record_audio_accepted_(const std::shared_ptr<ListenGoalHandle> goal_handle) {}

void ListenNode::record_audio_(const std::shared_ptr<ListenGoalHandle> goal_handle) {}
} // end of namespace whisper
