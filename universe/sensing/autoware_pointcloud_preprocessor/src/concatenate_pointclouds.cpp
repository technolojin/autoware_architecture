#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace autoware::pointcloud_preprocessor
{

class ConcatenatePointcloudComponent : public rclcpp::Node
{
public:
  explicit ConcatenatePointcloudComponent(const rclcpp::NodeOptions & options)
  : Node("ConcatenatePointcloudComponent", options)
  {
    // Create a timer that calls the callback every 2 seconds (1/2 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&ConcatenatePointcloudComponent::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "ConcatenatePointcloudComponent started");
  }

private:
  void timer_callback()
  {
    // Publish node namespace with node name
    std::string node_namespace = this->get_namespace();
    std::string node_name = this->get_name();
    std::string full_name = (node_namespace == "/") ? "/" + node_name : node_namespace + "/" + node_name;
    RCLCPP_INFO(this->get_logger(), "Node %s", full_name.c_str());
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::ConcatenatePointcloudComponent)
