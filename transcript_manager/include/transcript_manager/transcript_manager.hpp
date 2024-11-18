#ifndef TRANSCRIPT_MANAGER__TRANSCRIPT_MANAGER_HPP_
#define TRANSCRIPT_MANAGER__TRANSCRIPT_MANAGER_HPP_

#include <string>
#include <vector>
#include <memory>   // std::unique_ptr
#include <utility>  // std::pair

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Repo Messages/Actions
#include "whisper_idl/action/inference.hpp"
#include "whisper_idl/msg/whisper_tokens.hpp"
#include "whisper_idl/msg/audio_transcript.hpp" 

// Repo tools
#include "whisper_util/audio_buffers.hpp"
#include "whisper_util/chrono_utils.hpp"
#include "transcript_manager/tokens.hpp"
#include "transcript_manager/words.hpp"
#include "transcript_manager/segments.hpp"
#include "transcript_manager/transcript.hpp"

namespace whisper {

class TranscriptManager : public rclcpp::Node {
  using Inference = whisper_idl::action::Inference;
  using GoalHandleInference = rclcpp_action::ServerGoalHandle<Inference>;
  using WhisperTokens = whisper_idl::msg::WhisperTokens;
  using AudioTranscript = whisper_idl::msg::AudioTranscript;

  // Whisper gives duration info on segments which are related to ms by a ratio
  const int whisper_ts_to_ms_ratio = 10;
  // Setting for how many tokens within brackets (e.g. "[ .. ]") could be combined
  const int max_number_tokens_to_combine = 10;

public:
  TranscriptManager(const rclcpp::NodeOptions& options);

protected:
  // whisper output subscription
  rclcpp::Subscription<WhisperTokens>::SharedPtr tokens_sub_;
  void on_whisper_tokens_(const WhisperTokens::SharedPtr msg);
  std::vector<Segment> deserialize_msg_(const WhisperTokens::SharedPtr &msg);

  // action server
  rclcpp_action::Server<Inference>::SharedPtr inference_action_server_;
  rclcpp_action::GoalResponse on_inference_(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const Inference::Goal> goal);
  rclcpp_action::CancelResponse
        on_cancel_inference_(const std::shared_ptr<GoalHandleInference> goal_handle);
  void on_inference_accepted_(const std::shared_ptr<GoalHandleInference> goal_handle);
  rclcpp::Time inference_start_time_;

  // Callback to merge incoming queue into transcript
  bool clear_queue_();
  rclcpp::TimerBase::SharedPtr clear_queue_timer_;

  // Outgoing continuous audio transcription publishing
  void serialize_transcript_(AudioTranscript &msg);
  rclcpp::Publisher<AudioTranscript>::SharedPtr transcript_pub_;

private:
  // Data
  std::unique_ptr<ThreadSafeRing<std::vector<Segment>>> incoming_queue_;
  std::unique_ptr<Transcript> transcript_;

  // Helper functions for deseralizing the message
  bool is_special_token(const std::vector<std::string> &tokens, const int idx);
  bool my_ispunct(const std::vector<std::string> &tokens, const int idx);
  bool contains_char(const std::string &str, const char target); // helper
  std::pair<bool, int> join_tokens(const std::vector<std::string> &tokens, const int idx);
  std::string combine_text(const std::vector<std::string> &tokens, const int idx, const int num);
  float combine_prob(const std::vector<float> &probs, const int idx, const int num);

  // Print functions
  void print_msg_(const WhisperTokens::SharedPtr &msg);
  void print_new_words_(const std::vector<Segment> &new_words);
};
} // end of namespace whisper
#endif // TRANSCRIPT_MANAGER__TRANSCRIPT_MANAGER_HPP_
