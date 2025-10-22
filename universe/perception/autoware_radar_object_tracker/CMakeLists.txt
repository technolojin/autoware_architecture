cmake_minimum_required(VERSION 3.14)
project(autoware_radar_object_tracker)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library(radar_object_tracker_cpp SHARED
  src/node.cpp
)
rclcpp_components_register_node(radar_object_tracker_cpp
  PLUGIN "autoware::radar_object_tracker::RadarObjectTrackerNode"
  EXECUTABLE "radar_object_tracker_node"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  config
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
