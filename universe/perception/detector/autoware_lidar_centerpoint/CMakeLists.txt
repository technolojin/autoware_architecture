cmake_minimum_required(VERSION 3.14)
project(autoware_lidar_centerpoint)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()
