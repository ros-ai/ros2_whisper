#ifndef WHISPER_NODES__TRANSCRIPT_MANAGER_NODE_HPP_
#define WHISPER_NODES__TRANSCRIPT_MANAGER_NODE_HPP_

#include <chrono>
// #include <memory>
// #include <numeric>
// #include <stdexcept>
#include <string>
#include <vector>
#include <utility>  // For std::pair
// #include <mutex>

// #include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "std_msgs/msg/int16_multi_array.hpp"

#include "whisper_idl/action/inference.hpp"
#include "whisper_idl/msg/whisper_tokens.hpp"
#include "whisper_idl/msg/audio_transcript.hpp" 

#include "whisper_util/transcript_data.hpp"
#include "whisper_util/audio_buffers.hpp"

namespace whisper {

class TranscriptManagerNode {
  using Inference = whisper_idl::action::Inference;
  using GoalHandleInference = rclcpp_action::ServerGoalHandle<Inference>;
  using WhisperTokens = whisper_idl::msg::WhisperTokens;
  using AudioTranscript = whisper_idl::msg::AudioTranscript;
  using WordsAndSegments = std::pair<std::vector<Word>, std::vector<SegmentMetaData>>;


public:
  TranscriptManagerNode(const rclcpp::Node::SharedPtr node_ptr);

protected:
  rclcpp::Node::SharedPtr node_ptr_;

  // audio subscription
  rclcpp::Subscription<WhisperTokens>::SharedPtr tokens_sub_;
  void on_whisper_tokens_(const WhisperTokens::SharedPtr msg);
  std::pair<std::vector<Word>, std::vector<SegmentMetaData>> 
                              deserialize_msg_(const WhisperTokens::SharedPtr &msg);
  void print_msg_(const WhisperTokens::SharedPtr &msg);
  void print_new_words_(const std::vector<Word> &new_words,
                          const std::vector<SegmentMetaData> &new_segments);

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
  std::unique_ptr<ThreadSafeRing<WordsAndSegments>> incoming_queue_;
  void clear_queue_();
  void merge_one_(const WordsAndSegments &words_and_segments);
  // void _backup_merge_one_(const WordsAndSegments &words_and_segments);
  Transcript transcript_;

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
#endif // WHISPER_NODES__TRANSCRIPT_MANAGER_NODE_HPP_
