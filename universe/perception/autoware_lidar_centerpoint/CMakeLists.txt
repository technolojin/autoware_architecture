cmake_minimum_required(VERSION 3.14)
project(autoware_lidar_centerpoint)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library(lidar_centerpoint_cpp SHARED
  src/node.cpp
)
rclcpp_components_register_node(lidar_centerpoint_cpp
  PLUGIN "autoware::lidar_centerpoint::LidarCenterPointNode"
  EXECUTABLE "autoware_lidar_centerpoint_node"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
