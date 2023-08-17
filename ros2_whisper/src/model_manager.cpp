#include "ros2_whisper/model_manager.hpp"

namespace ros2_whisper {

ModelManager::ModelManager(const std::string &src, const std::string &pfx,
                           const std::string &cache_path)
    : src_(src), pfx_(pfx), cache_path_(cache_path) {
  mkdir(cache_path);
}

bool ModelManager::is_available(const std::string &model_name) {
  return std::filesystem::exists(cache_path_ / std::filesystem::path(model_name));
}

void ModelManager::make_available(const std::string &model_name) {
  using namespace std;
  cout << "***************************" << endl;
  cout << "Attempting model download: " << model_name << endl;
  cout << "From: " << src_ << endl;
  cout << "To: " << cache_path_ << endl;
  cout << "Continue? [y/n]";
  string answer;
  cin >> answer;
  if (answer != "y") {
    cout << "Aborting" << endl;
    return;
  }
  system(("wget --no-config --quiet --show-progress -P " + cache_path_.string() + " " + src_ + "/" +
          pfx_ + "/" + model_name)
             .c_str());
}

std::string ModelManager::get_model_path(const std::string &model_name) {
  if (!is_available(model_name)) {
    throw std::runtime_error("Model not available: " + model_name);
  }
  return (cache_path_ / std::filesystem::path(model_name)).string();
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

} // end of namespace ros2_whisper
