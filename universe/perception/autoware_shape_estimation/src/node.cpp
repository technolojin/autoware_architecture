#include <rclcpp/rclcpp.hpp>
#include <chrono>

class ShapeEstimationNode : public rclcpp::Node
{
public:
  ShapeEstimationNode()
  : Node("shape_estimation")
  {
    // Create a timer that calls the callback every 2 seconds (1/2 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&ShapeEstimationNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "ShapeEstimationNode started");
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShapeEstimationNode>());
  rclcpp::shutdown();
  return 0;
}
