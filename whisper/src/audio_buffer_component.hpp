#ifndef ROS2_WHISPER__AUDIO_BUFFER_COMPONENT_HPP_
#define ROS2_WHISPER__AUDIO_BUFFER_COMPONENT_HPP_

#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "whisper_msgs/srv/provide_audio.hpp"
#include "whisper_wrapper/audio_buffer.hpp"

namespace whisper {
class AudioBufferComponent : public rclcpp::Node {
public:
  AudioBufferComponent(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

protected:
  std::atomic_bool record_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  rclcpp::Service<whisper_msgs::srv::ProvideAudio>::SharedPtr provide_service_;

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr audio_subscription_;

  AudioBuffer audio_buffer_;
};
} // end of namespace whisper
#endif // ROS2_WHISPER__AUDIO_BUFFER_COMPONENT_HPP_
