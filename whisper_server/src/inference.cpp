#include "whisper_server/inference.hpp"

namespace whisper {
Inference::Inference(const rclcpp::NodeOptions& options)
    : Node("inference", options), language_("en") {
  declare_parameters_();

  // audio subscription:  Allow parallel callbacks
  auto cb_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group;
  audio_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      "audio", rclcpp::SensorDataQoS(), 
      std::bind(&Inference::on_audio_, this, std::placeholders::_1), sub_options);

  // parameter callback handle
  on_parameter_set_handle_ = add_on_set_parameters_callback(
      std::bind(&Inference::on_parameter_set_, this, std::placeholders::_1));

  // Data
  auto audio_ring_s_ = std::chrono::seconds(get_parameter("buffer_capacity").as_int());
  audio_ring_ = std::make_unique<AudioRing>(audio_ring_s_);

  // whisper
  model_manager_ = std::make_unique<ModelManager>();
  whisper_ = std::make_unique<Whisper>();
  initialize_whisper_();

  // Inference publisher 
  auto callback_ms = std::chrono::milliseconds(get_parameter("callback_ms").as_int());
  timer_ = create_wall_timer(callback_ms, std::bind(&Inference::timer_callback, this), cb_group);
  publisher_ = create_publisher<whisper_idl::msg::WhisperTokens>("tokens", 10);

  active_ = get_parameter("active").as_bool();
}

void Inference::timer_callback()
{
  if ( active_ ) {
    auto msg = create_message_();
    auto success = run_inference_(msg);
    if ( success ) {
      publisher_->publish(msg);

      auto& clk = *get_clock();
      RCLCPP_INFO_THROTTLE(get_logger(), clk, 5000,
                        "Whisper Induced Lag:   %ld (ms).",
                        msg.inference_duration);
    }
  }
}

void Inference::declare_parameters_() {
  // buffer parameters
  declare_parameter("buffer_capacity", 2);
  declare_parameter("callback_ms", 200);
  declare_parameter("active", false);

  // whisper parameters
  declare_parameter("model_name", "base.en");
  // consider other parameters:
  // https://github.com/ggerganov/whisper.cpp/blob/a4bb2df36aeb4e6cfb0c1ca9fbcf749ef39cc852/whisper.h#L351
  declare_parameter("wparams.language", "en");
  declare_parameter("wparams.n_threads", 4);
  declare_parameter("wparams.print_progress", false);
  declare_parameter("cparams.flash_attn", true);
  declare_parameter("cparams.gpu_device", 0);
  declare_parameter("cparams.use_gpu", true);
}

void Inference::initialize_whisper_() {
  std::string model_name = get_parameter("model_name").as_string();
  RCLCPP_INFO(get_logger(), "Checking whether model %s is available...",
              model_name.c_str());
  if ( !model_manager_->is_available(model_name) ) {
    RCLCPP_INFO(get_logger(), "Model %s is not available. Attempting download...",
                model_name.c_str());
    if ( model_manager_->make_available(model_name) != 0 ) {
      std::string err_msg = "Failed to download model " + model_name + ".";
      RCLCPP_ERROR(get_logger(), err_msg.c_str());
      throw std::runtime_error(err_msg);
    }
    RCLCPP_INFO(get_logger(), "Model %s downloaded.", model_name.c_str());
  }
  RCLCPP_INFO(get_logger(), "Model %s is available.", model_name.c_str());

  language_ = get_parameter("wparams.language").as_string();
  whisper_->wparams.language = language_.c_str();
  whisper_->wparams.n_threads = get_parameter("wparams.n_threads").as_int();
  whisper_->wparams.print_progress = get_parameter("wparams.print_progress").as_bool();
  whisper_->cparams.flash_attn = get_parameter("cparams.flash_attn").as_bool();
  whisper_->cparams.gpu_device = get_parameter("cparams.gpu_device").as_int();
  whisper_->cparams.use_gpu = get_parameter("cparams.use_gpu").as_bool();

  RCLCPP_INFO(get_logger(), "Initializing model %s...", model_name.c_str());
  whisper_->initialize(model_manager_->get_model_path(model_name));
  RCLCPP_INFO(get_logger(), "Model %s initialized.", model_name.c_str());
}

rcl_interfaces::msg::SetParametersResult
Inference::on_parameter_set_(const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto &parameter : parameters) {
    if ( parameter.get_name() == "n_threads" ) {
      whisper_->wparams.n_threads = parameter.as_int();
      RCLCPP_INFO(get_logger(), "Parameter %s set to %d.", parameter.get_name().c_str(),
                  whisper_->wparams.n_threads);
      continue;
    }
    if ( parameter.get_name() == "active" ) {
      active_ = parameter.as_bool();
      RCLCPP_INFO(get_logger(), "Parameter %s set to %d.", parameter.get_name().c_str(),
                  active_);
      continue;
    }
    result.reason = "Parameter " + parameter.get_name() + " not handled.";
    result.successful = false;
    RCLCPP_WARN(get_logger(), result.reason.c_str());
  }
  result.successful = true;
  return result;
}

void Inference::on_audio_(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
  if ( !audio_ring_->is_audio_start_set() ) {
    audio_ring_->set_start_timestamp(ros_time_to_chrono(get_clock()->now()));
  }
  // on_audio_debug_print_(msg);
  audio_ring_->enqueue(msg->data);
}

void Inference::inference_(const std::vector<float> &audio, 
                                  whisper_idl::msg::WhisperTokens &result) {
  auto inference_start_time = now();
  whisper_->forward_serialize(audio, 
                      result.token_ids, result.token_texts, result.token_probs,
                      result.segment_start_token_idxs, result.start_times, result.end_times);
  result.inference_duration =
      (now() - inference_start_time).to_chrono<std::chrono::milliseconds>().count();
  return;
}

whisper_idl::msg::WhisperTokens Inference::create_message_() {
  auto msg = whisper_idl::msg::WhisperTokens();
  msg.stamp = rclcpp::Clock().now();
  // TODO Reserve data based on previous token sizes
  return msg;
}

bool Inference::run_inference_(whisper_idl::msg::WhisperTokens &result) {
  if ( whisper_mutex_.try_lock() ) {
    const auto& [data, timestamp] = audio_ring_->peak();
    result.stamp = chrono_to_ros_msg(timestamp);

    inference_(data, result);

    // Print warning if inference takes too long for audio size
    auto duration = std::chrono::milliseconds(result.inference_duration);
    auto max_runtime_for_audio_size = whisper::count_to_time(data.size());
    if ( duration > max_runtime_for_audio_size ){
          auto timeout_duration_ms = max_runtime_for_audio_size.count();
          RCLCPP_WARN(get_logger(),
                "Inference took longer than audio buffer size. This leads to un-inferenced audio "
                "data. Consider increasing thread number or compile with accelerator support. \n "
                "\t Inference Duration:   %lld,  Timeout after  %lld", 
                static_cast<long long>(duration.count()), 
                static_cast<long long>(timeout_duration_ms));
    }
    whisper_mutex_.unlock();
    return true;
  } else {
    RCLCPP_INFO(get_logger(), "Whisper.cpp busy, skipping inference");
    return false;
  }
}

void Inference::on_audio_debug_print_(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
  auto audio_buff_start = audio_ring_->get_start_timestamp();
  auto audio_buff_len_ms = count_to_time(audio_ring_->size());
  auto cur_time = ros_time_to_chrono(get_clock()->now());
  auto msg_duration_ms = count_to_time(msg->data.size());
  size_t elapsed_count;
  bool negative;
  if ( (audio_buff_start + audio_buff_len_ms) > (cur_time - msg_duration_ms) ) {
    negative = true;
    elapsed_count = 
      time_to_count(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          (audio_buff_start + audio_buff_len_ms) - (cur_time - msg_duration_ms)));
  } else {
    negative = false;
    elapsed_count = 
      time_to_count(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          (cur_time - msg_duration_ms) - (audio_buff_start + audio_buff_len_ms)));
  }
  auto [audio_buff_start_s, audio_buff_start_ns] 
                  = chrono_time_to_ros(audio_buff_start);
  auto [audio_buff_end_s, audio_buff_end_ns] 
                  = chrono_time_to_ros(audio_buff_start + audio_buff_len_ms);
  auto [cur_s, cur_ns] = chrono_time_to_ros(cur_time);

  auto& clk = *get_clock();
  RCLCPP_DEBUG_THROTTLE(get_logger(), clk, 1000,
              "Audio Start:  %ld.%ld,  Audio End:  %ld.%ld,  Current time:  %ld.%ld\n"
              "Audio buffer length:  %ldms,    Message length:  %ldms\n"
              "Elapsed pad length:   %s%ld",
              audio_buff_start_s, audio_buff_start_ns,
              audio_buff_end_s, audio_buff_end_ns,
              cur_s, cur_ns,
              audio_buff_len_ms.count(), msg_duration_ms.count(),
              negative ? "-": "", elapsed_count);
}

} // end of namespace whisper


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(whisper::Inference)