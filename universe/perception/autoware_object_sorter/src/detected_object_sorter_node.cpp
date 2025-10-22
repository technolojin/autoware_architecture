#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace autoware::object_sorter
{
class DetectedObjectSorterNode : public rclcpp::Node
{
public:
  explicit DetectedObjectSorterNode(const rclcpp::NodeOptions & options)
  : Node("detected_object_sorter_node", options)
  {
    // Create a timer that calls the callback every 2 seconds (1/2 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&DetectedObjectSorterNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "DetectedObjectSorterNode started");
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

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::object_sorter::DetectedObjectSorterNode)
