cmake_minimum_required(VERSION 3.14)
project(autoware_compare_map_segmentation)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library(compare_map_segmentation_cpp SHARED
  src/voxel_based_compare_map_filter_node.cpp
)
rclcpp_components_register_node(compare_map_segmentation_cpp
  PLUGIN "autoware::compare_map_segmentation::VoxelBasedCompareMapFilterComponent"
  EXECUTABLE "voxel_based_compare_map_filter_node"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
