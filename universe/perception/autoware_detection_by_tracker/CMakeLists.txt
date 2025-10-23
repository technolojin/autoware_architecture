cmake_minimum_required(VERSION 3.14)
project(autoware_detection_by_tracker)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library(detection_by_tracker_cpp SHARED
  src/node.cpp
)
rclcpp_components_register_node(detection_by_tracker_cpp
  PLUGIN "autoware::detection_by_tracker::DetectionByTrackerNode"
  EXECUTABLE "detection_by_tracker_node"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
