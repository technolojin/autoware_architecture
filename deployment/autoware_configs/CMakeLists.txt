cmake_minimum_required(VERSION 3.14)
project(autoware_configs)

find_package(autoware_cmake REQUIRED)
find_package(autoware_architect REQUIRED)

autoware_package()

ament_auto_package(
  INSTALL_TO_SHARE
    config
)

autoware_architect_configure()
