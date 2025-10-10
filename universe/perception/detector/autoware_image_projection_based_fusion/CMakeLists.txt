cmake_minimum_required(VERSION 3.14)
project(autoware_image_projection_based_fusion)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()
