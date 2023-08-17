#ifndef WHISPER_NODES__WHISPER_NODE_HPP_
#define WHISPER_NODES__WHISPER_NODE_HPP_

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "whisper_msgs/srv/inference.hpp"
#include "whisper_wrapper/model_manager.hpp"
#include "whisper_wrapper/whisper.hpp"

namespace whisper {
class WhisperNode {
public:
  WhisperNode(const rclcpp::Node::SharedPtr node_ptr);

protected:
  void initialize_parameters_();
  void on_inference_(const whisper_msgs::srv::Inference::Request::SharedPtr request,
                     whisper_msgs::srv::Inference::Response::SharedPtr response);

  rclcpp::Node::SharedPtr node_ptr_;

  rclcpp::Service<whisper_msgs::srv::Inference>::SharedPtr inference_service_;

  ModelManager model_manager_;
  Whisper whisper_;
};
} // end of namespace whisper

#endif // WHISPER_NODES__WHISPER_NODE_HPP_
