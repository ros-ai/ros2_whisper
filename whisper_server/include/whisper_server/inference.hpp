#ifndef WHISPER_NODES__INFERENCE_NODE_HPP_
#define WHISPER_NODES__INFERENCE_NODE_HPP_

#include <chrono>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
// #include <mutex>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include "whisper_util/audio_buffers.hpp"
#include "whisper_util/model_manager.hpp"
#include "whisper_util/whisper.hpp"
#include "whisper_util/chrono_utils.hpp"

#include "whisper_idl/action/inference.hpp"
#include "whisper_idl/msg/whisper_tokens.hpp"

namespace whisper {

class Inference : public rclcpp::Node {
public:
  Inference(const rclcpp::NodeOptions& options);

protected:
  // parameters
  void declare_parameters_();
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_parameter_set_handle_;
  rcl_interfaces::msg::SetParametersResult
  on_parameter_set_(const std::vector<rclcpp::Parameter> &parameters);

  // audio subscription
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr audio_sub_;
  void on_audio_(const std_msgs::msg::Int16MultiArray::SharedPtr msg);

  // publsiher
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<whisper_idl::msg::WhisperTokens>::SharedPtr publisher_;
  whisper_idl::msg::WhisperTokens create_message_();

  // whisper
  std::unique_ptr<ModelManager> model_manager_;
  std::unique_ptr<Whisper> whisper_;
  std::mutex whisper_mutex_;
  std::string language_;
  void initialize_whisper_();
  
  bool run_inference_(whisper_idl::msg::WhisperTokens &result);
  void inference_(const std::vector<float> &audio, whisper_idl::msg::WhisperTokens &result);

private:
  // Data
  std::chrono::milliseconds update_ms_;
  std::unique_ptr<AudioRing> audio_ring_;

  // Control if whisper is running
  bool active_;

  // Helper/debug functions
  void on_audio_debug_print_(const std_msgs::msg::Int16MultiArray::SharedPtr msg);

};
} // end of namespace whisper
#endif // WHISPER_NODES__INFERENCE_NODE_HPP_
