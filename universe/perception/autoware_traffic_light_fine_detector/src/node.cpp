#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>

namespace autoware::traffic_light
{
class TrafficLightFineDetectorNode : public rclcpp::Node
{
public:
  explicit TrafficLightFineDetectorNode(const rclcpp::NodeOptions & options)
  : Node("traffic_light_fine_detector", options)
  {
    // Declare parameters
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<std::string>("label_path", "");
    this->declare_parameter<bool>("build_only", false);
    
    // Get parameters
    std::string model_path = this->get_parameter("model_path").as_string();
    std::string label_path = this->get_parameter("label_path").as_string();
    bool build_only = this->get_parameter("build_only").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "TrafficLightFineDetectorNode started");
    RCLCPP_INFO(this->get_logger(), "Model path: %s", model_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Label path: %s", label_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Build only: %s", build_only ? "true" : "false");
    
    // Create a timer that calls the callback every 2 seconds (1/2 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&TrafficLightFineDetectorNode::timer_callback, this));
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

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightFineDetectorNode)
