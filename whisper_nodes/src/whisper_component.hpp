#ifndef ROS2_WHISPER__WHISPER_COMPONENT_HPP_
#define ROS2_WHISPER__WHISPER_COMPONENT_HPP_

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "whisper_msgs/srv/inference.hpp"
#include "whisper_msgs/srv/provide_audio.hpp"
#include "whisper/model_manager.hpp"
#include "whisper/whisper.hpp"

namespace whisper {
class WhisperComponent : public rclcpp::Node {
public:
  WhisperComponent(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

protected:
  void initialize_parameters_();
  void on_inference_(const whisper_msgs::srv::Inference::Request::SharedPtr request,
                     whisper_msgs::srv::Inference::Response::SharedPtr response);

  rclcpp::Service<whisper_msgs::srv::Inference>::SharedPtr inference_service_; // action server!
  rclcpp::Client<whisper_msgs::srv::ProvideAudio>::SharedPtr provide_client_;

  ModelManager model_manager_;
  Whisper whisper_;
};
} // end of namespace whisper

#endif // ROS2_WHISPER__WHISPER_HPP_
