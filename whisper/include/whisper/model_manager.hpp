#ifndef WHISPER__MODEL_MANAGER_HPP_
#define WHISPER__MODEL_MANAGER_HPP_

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace whisper {

class ModelManager {
public:
  ModelManager(const std::string &src = "https://huggingface.co/ggerganov/whisper.cpp",
               const std::string &pfx = "resolve/main",
               const std::string &cache_path = std::string(std::getenv("HOME")) +
                                               "/.cache/whisper.cpp");
  void mkdir(const std::string &path);
  bool is_available(const std::string &model_name = "base.en");
  int make_available(const std::string &model_name = "base.en");
  std::string get_model_path(const std::string &model_name = "base.en");

protected:
  std::string model_name_to_file_name_(const std::string &model_name);

  std::string src_;
  std::string pfx_;
  std::filesystem::path cache_path_;
};

} // end of namespace whisper

#endif // WHISPER__MODEL_MANAGER_HPP_
