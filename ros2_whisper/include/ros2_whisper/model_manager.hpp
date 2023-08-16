#ifndef ROS2_WHISPER__MODEL_MANAGER_HPP_
#define ROS2_WHISPER__MODEL_MANAGER_HPP_

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace ros2_whisper {

class ModelManager {
public:
  ModelManager(const std::string &src = "https://huggingface.co/ggerganov/whisper.cpp",
               const std::string &pfx = "resolve/main",
               const std::string &cache_path = std::string(std::getenv("HOME")) +
                                               "/.cache/whisper.cpp");
  void mkdir(const std::string &path);
  bool is_available(const std::string &model_name = "ggml-base.en.bin");
  void make_available(const std::string &model_name = "ggml-base.en.bin");
  std::string get_model_path(const std::string &model_name = "ggml-base.en.bin");

protected:
  std::string src_;
  std::string pfx_;
  std::filesystem::path cache_path_;
};

} // end of namespace ros2_whisper

#endif // ROS2_WHISPER__MODEL_MANAGER_HPP_
