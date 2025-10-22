cmake_minimum_required(VERSION 3.14)
project(autoware_pointcloud_preprocessor)

find_package(autoware_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
autoware_package()

# Add executable node
ament_auto_add_library(crop_box_filter_node_cpp SHARED
  src/crop_box_filter.cpp
)
rclcpp_components_register_node(crop_box_filter_node_cpp
  PLUGIN "autoware::image_projection_based_fusion::CropBoxFilterNode"
  EXECUTABLE crop_box_filter_node
)

ament_auto_add_library(concatenate_pointcloud_node_cpp SHARED
  src/concatenate_pointclouds.cpp
)
rclcpp_components_register_node(concatenate_pointcloud_node_cpp
  PLUGIN "autoware::image_projection_based_fusion::ConcatenatePointcloudComponent"
  EXECUTABLE concatenate_pointcloud_node
)

ament_auto_add_library(pickup_based_voxel_grid_downsample_filter_node_cpp SHARED
  src/pickup_based_voxel_grid_downsample_filter_node.cpp
)
rclcpp_components_register_node(pickup_based_voxel_grid_downsample_filter_node_cpp
  PLUGIN "autoware::image_projection_based_fusion::PickupBasedVoxelGridDownsampleFilterComponent"
  EXECUTABLE pickup_based_voxel_grid_downsample_filter_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
