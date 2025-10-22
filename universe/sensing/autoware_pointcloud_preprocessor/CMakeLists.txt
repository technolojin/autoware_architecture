cmake_minimum_required(VERSION 3.14)
project(autoware_pointcloud_preprocessor)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add executable node
ament_auto_add_executable(crop_box_filter_node
  src/crop_box_filter.cpp
)

ament_auto_add_executable(concatenate_pointcloud_node
  src/concatenate_pointclouds.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
