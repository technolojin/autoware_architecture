#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace autoware::image_projection_based_fusion
{

class PointPaintingFusionNode : public rclcpp::Node
{
public:
  explicit PointPaintingFusionNode(const rclcpp::NodeOptions & options)
  : Node("point_painting_fusion", options)
  {
    // Create a timer that calls the callback every 2 seconds (1/2 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&PointPaintingFusionNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "PointPaintingFusionNode started");
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

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::image_projection_based_fusion::PointPaintingFusionNode)
