#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>

namespace autoware::lidar_centerpoint
{
class LidarCenterPointNode : public rclcpp::Node
{
public:
  explicit LidarCenterPointNode(const rclcpp::NodeOptions & options)
  : Node("autoware_lidar_centerpoint_node", options)
  {
    // Declare parameters
    this->declare_parameter<bool>("build_only", false);
    this->declare_parameter<std::string>("logger_name", "lidar_centerpoint");
    
    // Get parameters
    bool build_only = this->get_parameter("build_only").as_bool();
    std::string logger_name = this->get_parameter("logger_name").as_string();
    
    RCLCPP_INFO(this->get_logger(), "LidarCenterPointNode started");
    RCLCPP_INFO(this->get_logger(), "Build only: %s", build_only ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Logger name: %s", logger_name.c_str());
    
    // Create a timer that calls the callback every 2 seconds (1/2 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&LidarCenterPointNode::timer_callback, this));
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
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lidar_centerpoint::LidarCenterPointNode)
