cmake_minimum_required(VERSION 3.14)
project(autoware_object_merger)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library(object_merger_cpp SHARED
  src/node.cpp
)
rclcpp_components_register_node(object_merger_cpp
  PLUGIN "autoware::object_merger::ObjectAssociationMergerNode"
  EXECUTABLE "object_association_merger_node"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
