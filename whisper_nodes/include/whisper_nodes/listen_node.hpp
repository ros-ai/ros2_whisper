#ifndef WHISPER_NODES__LISTEN_NODE_HPP_
#define WHISPER_NODES__LISTEN_NODE_HPP_

#include <atomic>
#include <thread>

#include "builtin_interfaces/msg/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

#include "whisper_msgs/action/listen.hpp"
#include "whisper_wrapper/audio_buffer.hpp"

namespace whisper {
class ListenNode {
  using ListenAction = whisper_msgs::action::Listen;
  using ListenGoalHandle = rclcpp_action::ServerGoalHandle<ListenAction>;

public:
  ListenNode(const rclcpp::Node::SharedPtr node_ptr);

protected:
  rclcpp_action::GoalResponse on_listen_goal_(const rclcpp_action::GoalUUID &uuid,
                                                    const ListenAction::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse
  on_listen_cancel_(const std::shared_ptr<ListenGoalHandle> goal_handle);
  void on_listen_goal_accepted_(const std::shared_ptr<ListenGoalHandle> goal_handle);

  void record_audio_(const std::shared_ptr<ListenGoalHandle> goal_handle);

  rclcpp::Node::SharedPtr node_ptr_;

  std::atomic_bool recording_;
  rclcpp::Time start_time_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr audio_subscription_;
  rclcpp_action::Server<ListenAction>::SharedPtr record_audio_action_server_;

  AudioBuffer audio_buffer_;
};
} // end of namespace whisper
#endif // WHISPER_NODES__LISTEN_NODE_HPP_
