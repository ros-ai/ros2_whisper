#ifndef ROS2_WHISPER__INFERENCE_SERVER_HPP_
#define ROS2_WHISPER__INFERENCE_SERVER_HPP_

#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ros2_whisper/audio_buffer.hpp"

namespace ros2_whisper {
class InferenceServer {
public:
  InferenceServer(const rclcpp::Node::SharedPtr node_ptr);

protected:
  void on_inference(const std_srvs::srv::Trigger::Request::SharedPtr requset,
                    std_srvs::srv::Trigger::Response::SharedPtr response);

  rclcpp::Node::SharedPtr node_ptr_;

  std::atomic_bool record_audio_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_audio_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_audio_buffer_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_inference_service_;

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr audio_subscription_;

  AudioBuffer audio_buffer_;
};
} // end of namespace ros2_whisper

#endif // ROS2_WHISPER__INFERENCE_SERVER_HPP_
