cmake_minimum_required(VERSION 3.14)
project(autoware_ground_segmentation)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library(scan_ground_filter_cpp SHARED
  src/scan_ground_filter_node.cpp
)
rclcpp_components_register_node(scan_ground_filter_cpp
  PLUGIN "autoware::ground_segmentation::ScanGroundFilterComponent"
  EXECUTABLE "scan_ground_filter_node"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
