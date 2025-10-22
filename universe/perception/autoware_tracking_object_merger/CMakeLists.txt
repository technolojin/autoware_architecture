cmake_minimum_required(VERSION 3.14)
project(autoware_tracking_object_merger)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library(tracking_object_merger_cpp SHARED
  src/decorative_object_merger_node.cpp
)
rclcpp_components_register_node(tracking_object_merger_cpp
  PLUGIN "autoware::tracking_object_merger::DecorativeTrackerMergerNode"
  EXECUTABLE "decorative_object_merger_node"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
