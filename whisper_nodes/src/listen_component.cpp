#include "rclcpp/rclcpp.hpp"

#include "whisper_nodes/listen_node.hpp"

namespace whisper {
class ListenComponent {
public:
  ListenComponent(const rclcpp::NodeOptions &options)
      : node_ptr_(rclcpp::Node::make_shared("listen", options)), listen_node_(node_ptr_) {}

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
    return node_ptr_->get_node_base_interface();
  }

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  whisper::ListenNode listen_node_;
};
} // end of namespace whisper
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(whisper::ListenComponent)
