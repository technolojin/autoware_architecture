cmake_minimum_required(VERSION 3.14)
project(autoware_tensorrt_yolox)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add executable node
ament_auto_add_executable(autoware_tensorrt_yolox_node_exe
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
