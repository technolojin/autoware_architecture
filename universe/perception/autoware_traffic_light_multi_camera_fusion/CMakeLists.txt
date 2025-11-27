cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_multi_camera_fusion)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library(traffic_light_multi_camera_fusion_cpp SHARED
  src/node.cpp
)
rclcpp_components_register_node(traffic_light_multi_camera_fusion_cpp
  PLUGIN "autoware::traffic_light::MultiCameraFusion"
  EXECUTABLE "traffic_light_multi_camera_fusion_node"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
