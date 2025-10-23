#include <rclcpp/rclcpp.hpp>
#include <chrono>

class TrtYoloXNode : public rclcpp::Node
{
public:
  TrtYoloXNode()
  : Node("autoware_tensorrt_yolox_node_exe")
  {
    // Create a timer that calls the callback every 2 seconds (1/2 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&TrtYoloXNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "TrtYoloXNode started");
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrtYoloXNode>());
  rclcpp::shutdown();
  return 0;
}
