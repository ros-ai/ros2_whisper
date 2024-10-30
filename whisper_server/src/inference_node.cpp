#include "whisper_server/inference_node.hpp"

namespace whisper {
InferenceNode::InferenceNode(const rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_(node_ptr), language_("en") {
  declare_parameters_();

  // audio subscription:  Allow parallel callbacks
  auto cb_group = node_ptr_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = cb_group;
  audio_sub_ = node_ptr_->create_subscription<std_msgs::msg::Int16MultiArray>(
      "audio", rclcpp::SensorDataQoS(), 
      std::bind(&InferenceNode::on_audio_, this, std::placeholders::_1), options);

  // inference action server
  // inference_action_server_ = rclcpp_action::create_server<Inference>(
  //   node_ptr_, "inference",
  //   std::bind(&InferenceNode::on_inference_, this, std::placeholders::_1, std::placeholders::_2),
  //   std::bind(&InferenceNode::on_cancel_inference_, this, std::placeholders::_1),
  //   std::bind(&InferenceNode::on_inference_accepted_, this, std::placeholders::_1));

  // parameter callback handle
  on_parameter_set_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&InferenceNode::on_parameter_set_, this, std::placeholders::_1));

  // Data
  auto audio_ring_s_ = std::chrono::seconds(node_ptr_->get_parameter("buffer_capacity").as_int());
  audio_ring_ = std::make_unique<AudioRing>(audio_ring_s_);
  // update_ms_ = std::chrono::milliseconds(node_ptr_->get_parameter("update_ms").as_int());  

  // whisper
  model_manager_ = std::make_unique<ModelManager>();
  whisper_ = std::make_unique<Whisper>();
  initialize_whisper_();

  // Inference publisher -- Set as own callback group
  RCLCPP_INFO(node_ptr_->get_logger(), "Creating callback group");
  // auto inference_cb_group = 
  //       node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // doesnt work
  // auto inference_cb_group = 
  //       node_ptr_->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // doesnt work
  auto inference_cb_group = nullptr; // works
  // RCLCPP_INFO(node_ptr_->get_logger(), "Callback group created: %p", inference_cb_group.get());
  auto callback_ms = std::chrono::milliseconds(node_ptr_->get_parameter("callback_ms").as_int());
  RCLCPP_INFO(node_ptr_->get_logger(), "callback_ms:  %ld", callback_ms.count());

  timer_ = node_ptr_->create_wall_timer(
            callback_ms, std::bind(&InferenceNode::timer_callback, this), inference_cb_group);
  // timer_ = node_ptr_->create_wall_timer(
  //           callback_ms, std::bind(&InferenceNode::timer_callback, this));
  publisher_ = node_ptr_->create_publisher<whisper_idl::msg::WhisperTokens>("tokens", 10);
  active_ = node_ptr_->get_parameter("active").as_bool();


  // Manually add the callback group to the node's executor
  // rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_callback_group(inference_cb_group, node_ptr_->get_node_base_interface());

  // Initialize as dummy data
  last_success_timestamp = node_ptr_->now();
}


// int count = 0;
void InferenceNode::timer_callback()
{
  // int c = count;
  // count++;
  // RCLCPP_INFO(node_ptr_->get_logger(), "TIMER CALLBACK:  %d", c);
  if (true) {
    // rclcpp::sleep_for(std::chrono::milliseconds(1000));
    // RCLCPP_INFO(node_ptr_->get_logger(), " -- done:  %d", c);

    auto msg = create_message_();
    auto success = run_inference_(msg);
    if (success) {
      publisher_->publish(msg);

      // auto transcript = 
      //       std::accumulate(msg.token_texts.begin(), msg.token_texts.end(), std::string());
      // RCLCPP_INFO(node_ptr_->get_logger(), "Transcript: %s", transcript.c_str());

      auto& clk = *node_ptr_->get_clock();
      RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), clk, 5000,
                        "Audio Lag:   %ld (ms).",
                        msg.inference_duration);
    }
  }
}

void InferenceNode::declare_parameters_() {
  // buffer parameters
  node_ptr_->declare_parameter("buffer_capacity", 2);
  node_ptr_->declare_parameter("update_ms", 200);
  node_ptr_->declare_parameter("callback_ms", 200);
  node_ptr_->declare_parameter("active", false);

  // whisper parameters
  node_ptr_->declare_parameter("model_name", "base.en");
  // consider other parameters:
  // https://github.com/ggerganov/whisper.cpp/blob/a4bb2df36aeb4e6cfb0c1ca9fbcf749ef39cc852/whisper.h#L351
  node_ptr_->declare_parameter("wparams.language", "en");
  node_ptr_->declare_parameter("wparams.n_threads", 4);
  node_ptr_->declare_parameter("wparams.print_progress", false);
  node_ptr_->declare_parameter("cparams.flash_attn", true);
  node_ptr_->declare_parameter("cparams.gpu_device", 0);
  node_ptr_->declare_parameter("cparams.use_gpu", true);
}

void InferenceNode::initialize_whisper_() {
  std::string model_name = node_ptr_->get_parameter("model_name").as_string();
  RCLCPP_INFO(node_ptr_->get_logger(), "Checking whether model %s is available...",
              model_name.c_str());
  if (!model_manager_->is_available(model_name)) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Model %s is not available. Attempting download...",
                model_name.c_str());
    if (model_manager_->make_available(model_name) != 0) {
      std::string err_msg = "Failed to download model " + model_name + ".";
      RCLCPP_ERROR(node_ptr_->get_logger(), err_msg.c_str());
      throw std::runtime_error(err_msg);
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Model %s downloaded.", model_name.c_str());
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "Model %s is available.", model_name.c_str());

  language_ = node_ptr_->get_parameter("wparams.language").as_string();
  whisper_->wparams.language = language_.c_str();
  whisper_->wparams.n_threads = node_ptr_->get_parameter("wparams.n_threads").as_int();
  whisper_->wparams.print_progress = node_ptr_->get_parameter("wparams.print_progress").as_bool();
  whisper_->cparams.flash_attn = node_ptr_->get_parameter("cparams.flash_attn").as_bool();
  whisper_->cparams.gpu_device = node_ptr_->get_parameter("cparams.gpu_device").as_int();
  whisper_->cparams.use_gpu = node_ptr_->get_parameter("cparams.use_gpu").as_bool();

  RCLCPP_INFO(node_ptr_->get_logger(), "Initializing model %s...", model_name.c_str());
  whisper_->initialize(model_manager_->get_model_path(model_name));
  RCLCPP_INFO(node_ptr_->get_logger(), "Model %s initialized.", model_name.c_str());
}

rcl_interfaces::msg::SetParametersResult
InferenceNode::on_parameter_set_(const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto &parameter : parameters) {
    if (parameter.get_name() == "n_threads") {
      whisper_->wparams.n_threads = parameter.as_int();
      RCLCPP_INFO(node_ptr_->get_logger(), "Parameter %s set to %d.", parameter.get_name().c_str(),
                  whisper_->wparams.n_threads);
      continue;
    }
    if (parameter.get_name() == "active") {
      active_ = parameter.as_bool();
      RCLCPP_INFO(node_ptr_->get_logger(), "Parameter %s set to %d.", parameter.get_name().c_str(),
                  active_);
      continue;
    }
    result.reason = "Parameter " + parameter.get_name() + " not handled.";
    result.successful = false;
    RCLCPP_WARN(node_ptr_->get_logger(), result.reason.c_str());
  }
  result.successful = true;
  return result;
}



void InferenceNode::on_audio_debug_print_(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
  if (audio_ring_->almost_full()) {
    auto audio_buff_start = audio_ring_->get_start_timestamp();
    auto audio_buff_len_ms = count_to_time(audio_ring_->size());
    auto cur_time = ros_time_to_chrono(node_ptr_->get_clock()->now());
    auto msg_duration_ms = count_to_time(msg->data.size());
    size_t elapsed_count;
    bool negative;
    if ((audio_buff_start + audio_buff_len_ms) > (cur_time - msg_duration_ms)) {
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

    auto& clk = *node_ptr_->get_clock();
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 1000,
                "Audio Start:  %ld.%ld,  Audio End:  %ld.%ld,  Current time:  %ld.%ld\n"
                "Audio buffer length:  %ldms,    Message length:  %ldms\n"
                "Elapsed pad length:   %s%ld",
                audio_buff_start_s, audio_buff_start_ns,
                audio_buff_end_s, audio_buff_end_ns,
                cur_s, cur_ns,
                audio_buff_len_ms.count(), msg_duration_ms.count(),
                negative ? "-": "", elapsed_count);  
    
  }
}

void InferenceNode::on_audio_(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
  if (!audio_ring_->is_audio_start_set()) {
    audio_ring_->set_start_timestamp(ros_time_to_chrono(node_ptr_->get_clock()->now()));
  }
  // on_audio_debug_print_(msg);
  audio_ring_->enqueue(msg->data);
}

rclcpp_action::GoalResponse
InferenceNode::on_inference_(const rclcpp_action::GoalUUID & /*uuid*/,
                             std::shared_ptr<const Inference::Goal> /*goal*/) {
  RCLCPP_INFO(node_ptr_->get_logger(), "Received inference request.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
InferenceNode::on_cancel_inference_(const std::shared_ptr<GoalHandleInference> /*goal_handle*/) {
  RCLCPP_INFO(node_ptr_->get_logger(), "Cancelling inference...");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void InferenceNode::on_inference_accepted_(const std::shared_ptr<GoalHandleInference> goal_handle) {
  RCLCPP_INFO(node_ptr_->get_logger(), "Starting inference...");
  auto feedback = std::make_shared<Inference::Feedback>();
  auto result = std::make_shared<Inference::Result>();
  inference_start_time_ = node_ptr_->now();
  auto batch_idx = 0;
  while (rclcpp::ok()) {
    if (node_ptr_->now() - inference_start_time_ > goal_handle->get_goal()->max_duration) {
      result->info = "Inference timed out.";
      RCLCPP_INFO(node_ptr_->get_logger(), result->info.c_str());
      goal_handle->succeed(result);
      audio_ring_->clear();
      return;
    }

    if (goal_handle->is_canceling()) {
      result->info = "Inference cancelled.";
      RCLCPP_INFO(node_ptr_->get_logger(), result->info.c_str());
      goal_handle->canceled(result);
      audio_ring_->clear();
      return;
    }

    // run inference
    // auto [transcription, duration] = inference_(audio_ring_->peak());
    // if (duration > whisper::count_to_time(audio_ring_->size())) {
    //     auto timeout_duration_ms = whisper::count_to_time(audio_ring_->size()).count();
    //     RCLCPP_WARN(node_ptr_->get_logger(),
    //           "Inference took longer than audio buffer size. This leads to un-inferenced audio "
    //           "data. Consider increasing thread number or compile with accelerator support. \n "
    //           "\t Inference Duration:   %lld,  Timeout after  %lld", 
    //           static_cast<long long>(duration.count()), 
    //           static_cast<long long>(timeout_duration_ms));
    // }


    // // feedback to client
    // feedback->transcription = transcription;
    // feedback->batch_idx = batch_idx;
    // goal_handle->publish_feedback(feedback);
    // result->transcriptions.push_back(feedback->transcription);

    // // Sleep until next update
    // if (update_ms_ < duration) {
    //   auto& clk = *node_ptr_->get_clock();
    //   RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), clk, 10000,
    //                         "Whisper Inference Lag:   "
    //                         "Inference Duration:   %lldms,  Update step every  %lldms", 
    //                         static_cast<long long>(duration.count()), 
    //                         static_cast<long long>(update_ms_.count()));
    // }
    // else {
    //   std::this_thread::sleep_for(update_ms_ - duration);
    // }

    ++batch_idx;
  }

  if (rclcpp::ok()) {
    result->info = "Inference succeeded.";
    RCLCPP_INFO(node_ptr_->get_logger(), result->info.c_str());
    goal_handle->succeed(result);
    audio_ring_->clear();
  }
}


void InferenceNode::inference_(const std::vector<float> &audio, 
                                  whisper_idl::msg::WhisperTokens &result) {
  auto inference_start_time = node_ptr_->now();
  whisper_->forward_tokenize(audio, 
                      result.token_ids, result.token_texts, result.token_probs,
                      result.segment_start_token_idxs, result.start_times, result.end_times);
  result.inference_duration =
      (node_ptr_->now() - inference_start_time).to_chrono<std::chrono::milliseconds>().count();
  return;
}

whisper_idl::msg::WhisperTokens InferenceNode::create_message_() {
  auto msg = whisper_idl::msg::WhisperTokens();
  msg.stamp = rclcpp::Clock().now();
  // TODO Reserve data based on previous token sizes
  return msg;
}

bool InferenceNode::run_inference_(whisper_idl::msg::WhisperTokens &result) {
  // const auto &data = audio_ring_->peak();
  const auto& [data, timestamp] = audio_ring_->peak();
  auto [sec, nanosec] = chrono_time_to_ros(timestamp);
  result.stamp.sec = sec;
  result.stamp.nanosec = nanosec;
  // RCLCPP_INFO(node_ptr_->get_logger(), "Audio Inference Start:   %d.%d", 
  //                                result.stamp.sec, result.stamp.nanosec);


  // if (whisper_mutex_.try_lock()) {
  if (true) {
    last_success_timestamp = node_ptr_->now();
    inference_(data, result);
    // whisper_mutex_.unlock();

    // Print warning if inference takes too long for audio size
    auto duration = std::chrono::milliseconds(result.inference_duration);
    auto max_runtime_for_audio_size = whisper::count_to_time(data.size());
    if (duration > max_runtime_for_audio_size){
          auto timeout_duration_ms = max_runtime_for_audio_size.count();
          RCLCPP_WARN(node_ptr_->get_logger(),
                "Inference took longer than audio buffer size. This leads to un-inferenced audio "
                "data. Consider increasing thread number or compile with accelerator support. \n "
                "\t Inference Duration:   %lld,  Timeout after  %lld", 
                static_cast<long long>(duration.count()), 
                static_cast<long long>(timeout_duration_ms));
    }
    return true;
  }
  return false;
}

} // end of namespace whisper
