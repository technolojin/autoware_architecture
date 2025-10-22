#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace autoware::detected_object_validation
{
class ObstaclePointCloudBasedValidator : public rclcpp::Node
{
public:
  explicit ObstaclePointCloudBasedValidator(const rclcpp::NodeOptions & options)
  : Node("obstacle_pointcloud_based_validator_node", options)
  {
    // Create a timer that calls the callback every 2 seconds (1/2 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&ObstaclePointCloudBasedValidator::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "ObstaclePointCloudBasedValidator started");
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
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::detected_object_validation::ObstaclePointCloudBasedValidator)
