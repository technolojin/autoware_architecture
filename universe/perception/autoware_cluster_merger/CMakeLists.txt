cmake_minimum_required(VERSION 3.14)
project(autoware_cluster_merger)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library(cluster_merger_cpp SHARED
  src/node.cpp
)
rclcpp_components_register_node(cluster_merger_cpp
  PLUGIN "autoware::cluster_merger::ClusterMergerNode"
  EXECUTABLE "cluster_merger_node"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
