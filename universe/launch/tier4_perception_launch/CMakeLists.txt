cmake_minimum_required(VERSION 3.14)
project(tier4_perception_launch)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  deployment
  launch
)

autoware_architect_configure()

# Add a dummy library to make a target
add_library(${PROJECT_NAME} SHARED src/empty.cpp)

# Call the function to build and deploy the modules
autoware_architect_build_deploy(${PROJECT_NAME} universe_perception_clr_centerpoint_serial.deployment)
autoware_architect_build_deploy(${PROJECT_NAME} universe_perception_clr_pointpainting_serial.deployment)
autoware_architect_build_deploy(${PROJECT_NAME} universe_perception_clr_centerpoint_parallel.deployment)
autoware_architect_build_deploy(${PROJECT_NAME} universe_perception_cl_pointpainting.deployment)
autoware_architect_build_deploy(${PROJECT_NAME} universe_perception_cl_centerpoint.deployment)
autoware_architect_build_deploy(${PROJECT_NAME} universe_perception_cl_apollo.deployment)
autoware_architect_build_deploy(${PROJECT_NAME} universe_perception_lidar.deployment)
