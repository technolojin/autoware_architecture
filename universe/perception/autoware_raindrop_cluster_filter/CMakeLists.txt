cmake_minimum_required(VERSION 3.14)
project(autoware_raindrop_cluster_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library(low_intensity_cluster_filter_cpp SHARED
  src/low_intensity_cluster_filter_node.cpp
)
rclcpp_components_register_node(low_intensity_cluster_filter_cpp
  PLUGIN "autoware::raindrop_cluster_filter::LowIntensityClusterFilterNode"
  EXECUTABLE "low_intensity_cluster_filter_node"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
