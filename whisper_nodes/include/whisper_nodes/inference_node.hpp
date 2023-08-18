#ifndef WHISPER_NODES__INFERENCE_NODE_HPP_
#define WHISPER_NODES__INFERENCE_NODE_HPP_

#include <chrono>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

#include "whisper_msgs/action/listen.hpp"
#include "whisper_msgs/srv/inference.hpp"
#include "whisper_wrapper/model_manager.hpp"
#include "whisper_wrapper/whisper.hpp"

namespace whisper {
class InferenceNode {
public:
  InferenceNode(const rclcpp::Node::SharedPtr node_ptr);

protected:
  void declare_parameters_();
  void initialize_whisper_();
  void on_listen_feedback_(const whisper_msgs::action::Listen_FeedbackMessage::SharedPtr msg);
  void on_inference_(const whisper_msgs::srv::Inference::Request::SharedPtr request,
                     whisper_msgs::srv::Inference::Response::SharedPtr response);
  rcl_interfaces::msg::SetParametersResult
  on_parameter_set_(const std::vector<rclcpp::Parameter> &parameters);

  rclcpp::Node::SharedPtr node_ptr_;

  rclcpp::Subscription<whisper_msgs::action::Listen_FeedbackMessage>::SharedPtr
      listen_feedback_subscription_;
  rclcpp::Service<whisper_msgs::srv::Inference>::SharedPtr inference_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_parameter_set_handle_;

  ModelManager model_manager_;
  Whisper whisper_;
  std::string language_;
};
} // end of namespace whisper
#endif // WHISPER_NODES__INFERENCE_NODE_HPP_
