#include "rclcpp/rclcpp.hpp"

#include "whisper_nodes/audio_buffer_node.hpp"

namespace whisper {
class AudioBufferComponent {
public:
  AudioBufferComponent(const rclcpp::NodeOptions &options)
      : node_ptr_(rclcpp::Node::make_shared("audio_buffer", options)),
        audio_buffer_node_(node_ptr_) {}

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
    return node_ptr_->get_node_base_interface();
  }

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  whisper::AudioBufferNode audio_buffer_node_;
};
} // end of namespace whisper
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(whisper::AudioBufferComponent)
