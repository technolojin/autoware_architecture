cmake_minimum_required(VERSION 3.14)
project(autoware_detected_object_validation)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library(detected_object_validation_cpp SHARED
  src/object_lanelet_filter_node.cpp
)
rclcpp_components_register_node(detected_object_validation_cpp
  PLUGIN "autoware::detected_object_validation::DetectedObjectLaneletFilterNode"
  EXECUTABLE "object_lanelet_filter_node"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
