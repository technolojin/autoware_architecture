cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_map_based_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library(traffic_light_map_based_detector_cpp SHARED
  src/node.cpp
)
rclcpp_components_register_node(traffic_light_map_based_detector_cpp
  PLUGIN "autoware::traffic_light::MapBasedDetector"
  EXECUTABLE "traffic_light_map_based_detector"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
