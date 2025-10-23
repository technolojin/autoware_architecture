#include <rclcpp/rclcpp.hpp>
#include <chrono>

class VoxelGridBasedEuclideanClusterNode : public rclcpp::Node
{
public:
  VoxelGridBasedEuclideanClusterNode()
  : Node("voxel_grid_based_euclidean_cluster_node")
  {
    // Create a timer that calls the callback every 2 seconds (1/2 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&VoxelGridBasedEuclideanClusterNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "VoxelGridBasedEuclideanClusterNode started");
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
  rclcpp::spin(std::make_shared<VoxelGridBasedEuclideanClusterNode>());
  rclcpp::shutdown();
  return 0;
}
