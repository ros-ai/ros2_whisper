#ifndef WHISPER_NODES__INFERENCE_NODE_HPP_
#define WHISPER_NODES__INFERENCE_NODE_HPP_

#include <chrono>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>

#include "action_msgs/msg/goal_status.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include "whisper_msgs/action/inference.hpp"
#include "whisper_util/audio_buffers.hpp"
#include "whisper_util/model_manager.hpp"
#include "whisper_util/whisper.hpp"

namespace whisper {
class InferenceNode {
  using Inference = whisper_msgs::action::Inference;
  using GoalHandleInference = rclcpp_action::ServerGoalHandle<Inference>;

public:
  InferenceNode(const rclcpp::Node::SharedPtr node_ptr);

protected:
  void declare_parameters_();
  void initialize_whisper_();

  // paramters
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_parameter_set_handle_;
  rcl_interfaces::msg::SetParametersResult
  on_parameter_set_(const std::vector<rclcpp::Parameter> &parameters);

  // audio subscription
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr audio_sub_;
  void on_audio_(const std_msgs::msg::Int16MultiArray::SharedPtr msg);

  // action server
  rclcpp_action::Server<Inference>::SharedPtr inference_action_server_;
  rclcpp_action::GoalResponse on_listen_(const rclcpp_action::GoalUUID &uuid,
                                         std::shared_ptr<const Inference::Goal> goal);
  rclcpp_action::CancelResponse on_cancel_(const std::shared_ptr<GoalHandleInference> goal_handle);
  void on_accepted_(const std::shared_ptr<GoalHandleInference> goal_handle);

  rclcpp::Node::SharedPtr node_ptr_;
  ModelManager model_manager_;
  EpisodicBuffer episodic_buffer_;
  Whisper whisper_;
  std::string language_;
};
} // end of namespace whisper
#endif // WHISPER_NODES__INFERENCE_NODE_HPP_
