cmake_minimum_required(VERSION 3.14)
project(autoware_multi_object_tracker)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add executable node
ament_auto_add_executable(multi_object_tracker_node
  src/node.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
