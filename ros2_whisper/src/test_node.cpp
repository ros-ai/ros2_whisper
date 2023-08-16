#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include "ros2_whisper/inference_server.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_node");

  ros2_whisper::InferenceServer inference_server(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}