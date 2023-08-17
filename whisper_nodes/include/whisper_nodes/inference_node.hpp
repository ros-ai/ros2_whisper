#ifndef WHISPER_NODES__INFERENCE_NODE_HPP_
#define WHISPER_NODES__INFERENCE_NODE_HPP_

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

#include "whisper_msgs/srv/inference.hpp"
#include "whisper_wrapper/model_manager.hpp"
#include "whisper_wrapper/whisper.hpp"

namespace whisper {
class InferenceNode {
public:
  InferenceNode(const rclcpp::Node::SharedPtr node_ptr);

protected:
  void declare_parameters_();
  void on_inference_(const whisper_msgs::srv::Inference::Request::SharedPtr request,
                     whisper_msgs::srv::Inference::Response::SharedPtr response);
  rcl_interfaces::msg::SetParametersResult
  on_parameter_set_(const std::vector<rclcpp::Parameter> &parameters);

  rclcpp::Node::SharedPtr node_ptr_;

  rclcpp::Service<whisper_msgs::srv::Inference>::SharedPtr inference_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_parameter_set_handle_;

  ModelManager model_manager_;
  Whisper whisper_;
};
} // end of namespace whisper
#endif // WHISPER_NODES__INFERENCE_NODE_HPP_
