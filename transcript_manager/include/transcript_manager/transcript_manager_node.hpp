#ifndef WHISPER_NODES__TRANSCRIPT_MANAGER_NODE_HPP_
#define WHISPER_NODES__TRANSCRIPT_MANAGER_NODE_HPP_

#include <chrono>
// #include <memory>
// #include <numeric>
// #include <stdexcept>
#include <string>
// #include <mutex>

// #include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "std_msgs/msg/int16_multi_array.hpp"

#include "whisper_idl/action/inference.hpp"
#include "whisper_idl/msg/whisper_tokens.hpp"

namespace whisper {
using namespace std::chrono_literals;
class TranscriptManagerNode {
  using Inference = whisper_idl::action::Inference;
  using GoalHandleInference = rclcpp_action::ServerGoalHandle<Inference>;
  using WhisperTokens = whisper_idl::msg::WhisperTokens;

public:
  TranscriptManagerNode(const rclcpp::Node::SharedPtr node_ptr);

protected:
  rclcpp::Node::SharedPtr node_ptr_;

  // audio subscription
  rclcpp::Subscription<WhisperTokens>::SharedPtr tokens_sub_;
  void on_whisper_tokens_(const WhisperTokens::SharedPtr msg);

  // action server
  rclcpp_action::Server<Inference>::SharedPtr inference_action_server_;
  rclcpp_action::GoalResponse on_inference_(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const Inference::Goal> goal);
  rclcpp_action::CancelResponse
        on_cancel_inference_(const std::shared_ptr<GoalHandleInference> goal_handle);
  void on_inference_accepted_(const std::shared_ptr<GoalHandleInference> goal_handle);
  rclcpp::Time inference_start_time_;

  // Data
  mutable std::string last_msg;
};
} // end of namespace whisper
#endif // WHISPER_NODES__TRANSCRIPT_MANAGER_NODE_HPP_
