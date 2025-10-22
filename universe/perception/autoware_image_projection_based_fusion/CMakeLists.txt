cmake_minimum_required(VERSION 3.14)
project(autoware_image_projection_based_fusion)

find_package(autoware_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
autoware_package()

# Add shared library for composable node plugin
ament_auto_add_library(roi_cluster_fusion_node_cpp SHARED
  src/roi_cluster_fusion_node.cpp
)

rclcpp_components_register_node(roi_cluster_fusion_node_cpp
  PLUGIN "autoware::image_projection_based_fusion::RoiClusterFusionNode"
  EXECUTABLE roi_cluster_fusion_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
