#include "whisper_wrapper/model_manager.hpp"

namespace whisper {

ModelManager::ModelManager(const std::string &src, const std::string &pfx,
                           const std::string &cache_path)
    : src_(src), pfx_(pfx), cache_path_(cache_path) {
  mkdir(cache_path);
}

bool ModelManager::is_available(const std::string &model_name) {
  return std::filesystem::exists(cache_path_ /
                                 std::filesystem::path(model_name_to_file_name_(model_name)));
}

int ModelManager::make_available(const std::string &model_name) {
  using namespace std;
  string url = src_ + "/" + pfx_ + "/" + model_name_to_file_name_(model_name);
  return system(
      ("wget --no-config --quiet -P " + cache_path_.string() + " " + url).c_str());
}

std::string ModelManager::get_model_path(const std::string &model_name) {
  if (!is_available(model_name)) {
    throw std::runtime_error("Model not available: " + model_name);
  }
  return (cache_path_ / std::filesystem::path(model_name_to_file_name_(model_name))).string();
}

void ModelManager::mkdir(const std::string &path) {
  using namespace std;
  if (filesystem::exists(path)) {
    cout << "Path already exists: " << path << endl;
    return;
  }
  cout << "Creating path: " << path << endl;
  filesystem::create_directories(path);
}

std::string ModelManager::model_name_to_file_name_(const std::string &model_name) {
  return "ggml-" + model_name + ".bin";
}

} // end of namespace whisper
