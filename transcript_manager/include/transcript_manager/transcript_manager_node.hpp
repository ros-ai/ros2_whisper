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
#include "whisper_util/chrono_utils.hpp"
#include "transcript_manager/tokens_and_segments.hpp"
#include "transcript_manager/words.hpp"
#include "transcript_manager/transcript.hpp"

namespace whisper {

class TranscriptManagerNode {
  using Inference = whisper_idl::action::Inference;
  using GoalHandleInference = rclcpp_action::ServerGoalHandle<Inference>;
  using WhisperTokens = whisper_idl::msg::WhisperTokens;
  using AudioTranscript = whisper_idl::msg::AudioTranscript;

  const int whisper_ts_to_ms_ratio = 10;
  // Setting for how many tokens within brackets (e.g. "[ .. ]") could be combined
  const int max_number_tokens_to_combine = 10;


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
  std::unique_ptr<Transcript> transcript_;
  void clear_queue_();

  void serialize_transcript_(AudioTranscript &msg);

private:
  // Helper functions for deseralizing the message
  bool is_special_token(const std::vector<std::string> &tokens, const int idx);
  bool my_ispunct(const std::vector<std::string> &tokens, const int idx);
  bool contains_char(const std::string &str, const char target); // helper
  std::pair<bool, int> join_tokens(const std::vector<std::string> &tokens, const int idx);
  std::string combine_text(const std::vector<std::string> &tokens, const int idx, const int num);
  float combine_prob(const std::vector<float> &probs, const int idx, const int num);
};
} // end of namespace whisper
#endif // TRANSCRIPT_MANAGER__TRANSCRIPT_MANAGER_NODE_HPP_
