#ifndef WHISPER_NODES__AUDIO_BUFFER_NODE_HPP_
#define WHISPER_NODES__AUDIO_BUFFER_NODE_HPP_

#include <atomic>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

#include "whisper_msgs/action/record_audio.hpp"
#include "whisper_wrapper/audio_buffer.hpp"

namespace whisper {
class AudioBufferNode {
  using RecordAudioAction = whisper_msgs::action::RecordAudio;
  using RecordAudioGoalHandle = rclcpp_action::ServerGoalHandle<RecordAudioAction>;

public:
  AudioBufferNode(const rclcpp::Node::SharedPtr node_ptr);

protected:
  rclcpp_action::GoalResponse
  on_record_audio_goal_(const rclcpp_action::GoalUUID &uuid,
                        const RecordAudioAction::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse
  on_record_audio_cancel_(const std::shared_ptr<RecordAudioGoalHandle> goal_handle);
  void on_record_audio_accepted_(const std::shared_ptr<RecordAudioGoalHandle> goal_handle);

  void record_audio_(const std::shared_ptr<RecordAudioGoalHandle> goal_handle);

  rclcpp::Node::SharedPtr node_ptr_;

  std::atomic_bool record_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr audio_subscription_;
  rclcpp_action::Server<RecordAudioAction>::SharedPtr record_audio_action_server_;

  AudioBuffer audio_buffer_;
};
} // end of namespace whisper
#endif // WHISPER_NODES__AUDIO_BUFFER_NODE_HPP_
