#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace autoware::image_projection_based_fusion
{

class RoiPointCloudFusionNode : public rclcpp::Node
{
public:
  explicit RoiPointCloudFusionNode(const rclcpp::NodeOptions & options)
  : Node("roi_point_cloud_fusion", options)
  {
    // Create a timer that calls the callback every 2 seconds (1/2 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&RoiPointCloudFusionNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "RoiPointCloudFusionNode started");
  }

private:
  void timer_callback()
  {
    // Publish node namespace with node name
    std::string node_namespace = this->get_namespace();
    std::string node_name = this->get_name();
    RCLCPP_INFO(this->get_logger(), "Node %s %s", node_namespace.c_str(), node_name.c_str());
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autoware::image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::image_projection_based_fusion::RoiPointCloudFusionNode)
