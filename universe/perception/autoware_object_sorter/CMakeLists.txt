cmake_minimum_required(VERSION 3.14)
project(autoware_object_sorter)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library(detected_object_sorter_cpp SHARED
  src/detected_object_sorter_node.cpp
)
rclcpp_components_register_node(detected_object_sorter_cpp
  PLUGIN "autoware::object_sorter::DetectedObjectSorterNode"
  EXECUTABLE "detected_object_sorter_node"
)

ament_auto_add_library(tracked_object_sorter_cpp SHARED
  src/tracked_object_sorter_node.cpp
)
rclcpp_components_register_node(tracked_object_sorter_cpp
  PLUGIN "autoware::object_sorter::TrackedObjectSorterNode"
  EXECUTABLE "tracked_object_sorter_node"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
