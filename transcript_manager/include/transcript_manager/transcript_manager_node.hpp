#ifndef TRANSCRIPT_MANAGER__TRANSCRIPT_MANAGER_NODE_HPP_
#define TRANSCRIPT_MANAGER__TRANSCRIPT_MANAGER_NODE_HPP_

#include <chrono>
// #include <memory>
// #include <numeric>
// #include <stdexcept>
#include <string>
#include <vector>
#include <utility>  // std::pair
// #include <mutex>

// #include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <builtin_interfaces/msg/time.hpp>
// #include "std_msgs/msg/int16_multi_array.hpp"

#include "whisper_idl/action/inference.hpp"
#include "whisper_idl/msg/whisper_tokens.hpp"
#include "whisper_idl/msg/audio_transcript.hpp" 

#include "whisper_util/audio_buffers.hpp"
#include "transcript_manager/tokens_and_segments.hpp"
#include "transcript_manager/words.hpp"
#include "transcript_manager/transcript.hpp"

namespace whisper {

inline std::chrono::system_clock::time_point ros_msg_to_chrono(
                      const builtin_interfaces::msg::Time& ros_time) {
  // Convert ROS time (seconds and nanoseconds) to a std::chrono::duration
  std::chrono::seconds sec(ros_time.sec);
  std::chrono::nanoseconds nsec(ros_time.nanosec);
  std::chrono::system_clock::duration total_duration = sec + nsec;

  std::chrono::system_clock::time_point time_point(total_duration);
  return time_point;
};

inline builtin_interfaces::msg::Time chrono_to_ros_msg(
                      const std::chrono::system_clock::time_point& timestamp) {
  builtin_interfaces::msg::Time ros_time;

  std::chrono::system_clock::duration duration_since_epoch = timestamp.time_since_epoch();
  std::chrono::seconds sec = 
    std::chrono::duration_cast<std::chrono::seconds>(duration_since_epoch);
  std::chrono::nanoseconds nano = 
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epoch - sec);

  ros_time.sec = sec.count();
  ros_time.nanosec = nano.count();
  return ros_time;
};

class TranscriptManagerNode {
  using Inference = whisper_idl::action::Inference;
  using GoalHandleInference = rclcpp_action::ServerGoalHandle<Inference>;
  using WhisperTokens = whisper_idl::msg::WhisperTokens;
  using AudioTranscript = whisper_idl::msg::AudioTranscript;

  const int whisper_ts_to_ms_ratio = 10;


public:
  TranscriptManagerNode(const rclcpp::Node::SharedPtr node_ptr);

protected:
  rclcpp::Node::SharedPtr node_ptr_;

  // audio subscription
  rclcpp::Subscription<WhisperTokens>::SharedPtr tokens_sub_;
  void on_whisper_tokens_(const WhisperTokens::SharedPtr msg);
  std::vector<Word> deserialize_msg_(const WhisperTokens::SharedPtr &msg);
  void print_msg_(const WhisperTokens::SharedPtr &msg);
  void print_new_words_(const std::vector<Word> &new_words_and_segments);
  void print_timestamp_(std::chrono::system_clock::time_point timestamp);

  // action server
  rclcpp_action::Server<Inference>::SharedPtr inference_action_server_;
  rclcpp_action::GoalResponse on_inference_(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const Inference::Goal> goal);
  rclcpp_action::CancelResponse
        on_cancel_inference_(const std::shared_ptr<GoalHandleInference> goal_handle);
  void on_inference_accepted_(const std::shared_ptr<GoalHandleInference> goal_handle);
  rclcpp::Time inference_start_time_;

  // Clear Incmoing Queue timer
  void clear_queue_callback_();
  rclcpp::TimerBase::SharedPtr clear_queue_timer_;

  // Outgoing continuous audio transcription publishing
  rclcpp::Publisher<AudioTranscript>::SharedPtr transcript_pub_;

  // Data
  std::unique_ptr<ThreadSafeRing<std::vector<Word>>> incoming_queue_;
  void clear_queue_();
  void merge_one_(const std::vector<Word> &new_words);
  Transcript transcript_;

  void serialize_transcript_(AudioTranscript &msg);

  // LCS Hyperparameter
  int allowed_gaps;

  // Algorithms
  struct DPEntry {
      int length;
      int gaps;
  };
  std::tuple<std::vector<int>, std::vector<int>> lcs_indicies_(
                                                  const std::vector<std::string>& textA,
                                                  const std::vector<std::string>& textB,
                                                  int allowedGaps);

};
} // end of namespace whisper
#endif // TRANSCRIPT_MANAGER__TRANSCRIPT_MANAGER_NODE_HPP_
