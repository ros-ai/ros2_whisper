#ifndef ROS2_WHISPER__WHISPER_HPP_
#define ROS2_WHISPER__WHISPER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "whisper.h"

#include "ros2_whisper/model_manager.hpp"

namespace ros2_whisper {
class Whisper {
public:
  Whisper(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface);
  ~Whisper();

  std::vector<std::string> forward(const std::vector<float> &input);

protected:
  void initialize_parameters_();
  void initialize_model_();

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface_;

  whisper_context *wctx_;
  whisper_full_params wparams_;

  ModelManager model_manager_;
};
} // end of namespace ros2_whisper

#endif // ROS2_WHISPER__WHISPER_HPP_
