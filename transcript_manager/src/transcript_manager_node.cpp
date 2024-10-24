#include "transcript_manager/transcript_manager_node.hpp"

namespace whisper {
TranscriptManagerNode::TranscriptManagerNode(const rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_(node_ptr) {

  // Subscribe to incoming token data
  auto cb_group = node_ptr_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = cb_group;
  tokens_sub_ = node_ptr_->create_subscription<WhisperTokens>(
    "tokens", 10, 
    std::bind(&TranscriptManagerNode::on_whisper_tokens_, this, std::placeholders::_1), options);

  // Action Server
  inference_action_server_ = rclcpp_action::create_server<Inference>(
    node_ptr_, "inference",
    std::bind(&TranscriptManagerNode::on_inference_, this, 
                                          std::placeholders::_1, std::placeholders::_2),
    std::bind(&TranscriptManagerNode::on_cancel_inference_, this, std::placeholders::_1),
    std::bind(&TranscriptManagerNode::on_inference_accepted_, this, std::placeholders::_1));

  last_msg = "";
}

void TranscriptManagerNode::on_whisper_tokens_(const WhisperTokens::SharedPtr msg) {
  last_msg = 
      std::accumulate(msg->token_texts.begin(), msg->token_texts.end(), std::string());
  // RCLCPP_INFO(node_ptr_->get_logger(), "Transcript: %s", last_msg.c_str());
}

rclcpp_action::GoalResponse TranscriptManagerNode::on_inference_(
                              const rclcpp_action::GoalUUID & /*uuid*/,
                             std::shared_ptr<const Inference::Goal> /*goal*/) {
  RCLCPP_INFO(node_ptr_->get_logger(), "Received inference request.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TranscriptManagerNode::on_cancel_inference_(
            const std::shared_ptr<GoalHandleInference> /*goal_handle*/) {
  RCLCPP_INFO(node_ptr_->get_logger(), "Cancelling inference...");
  return rclcpp_action::CancelResponse::ACCEPT;
}


void TranscriptManagerNode::on_inference_accepted_(
                          const std::shared_ptr<GoalHandleInference> goal_handle) {
  RCLCPP_INFO(node_ptr_->get_logger(), "Starting inference...");
  auto feedback = std::make_shared<Inference::Feedback>();
  auto result = std::make_shared<Inference::Result>();
  inference_start_time_ = node_ptr_->now();
  auto batch_idx = 0;
  std::string last_sent_msg = last_msg;
  while (rclcpp::ok()) {
    if (node_ptr_->now() - inference_start_time_ > goal_handle->get_goal()->max_duration) {
      result->info = "Inference timed out.";
      RCLCPP_INFO(node_ptr_->get_logger(), result->info.c_str());
      goal_handle->succeed(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      result->info = "Inference cancelled.";
      RCLCPP_INFO(node_ptr_->get_logger(), result->info.c_str());
      goal_handle->canceled(result);
      return;
    }

    feedback->transcription = last_msg;
    feedback->batch_idx = batch_idx;
    goal_handle->publish_feedback(feedback);
    result->transcriptions.push_back(feedback->transcription);


    // RCLCPP_INFO(node_ptr_->get_logger(), "Batch %d", batch_idx);
    last_sent_msg = last_msg;
    // Wait for update in other thread
    while (last_sent_msg == last_msg) {
      rclcpp::sleep_for(15ms);
    }
    ++batch_idx;
  }

  if (rclcpp::ok()) {
    result->info = "Inference succeeded.";
    RCLCPP_INFO(node_ptr_->get_logger(), result->info.c_str());
    goal_handle->succeed(result);
  }
}
} // end of namespace whisper
