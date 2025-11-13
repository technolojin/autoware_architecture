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

autoware_architect_configure(tier4_perception_launch)

# Add a dummy library to make a target
add_library(${PROJECT_NAME} SHARED src/empty.cpp)

# Call the function to build and deploy the modules
autoware_architect_build_deploy(${PROJECT_NAME} CameraLidarApollo.system dummy_modules;tier4_perception_launch)
autoware_architect_build_deploy(${PROJECT_NAME} CameraLidarCenterpoint.system dummy_modules;tier4_perception_launch)
autoware_architect_build_deploy(${PROJECT_NAME} CameraLidarPointpainting.system dummy_modules;tier4_perception_launch)
autoware_architect_build_deploy(${PROJECT_NAME} CameraLidarRadarCenterpointParallel.system dummy_modules;tier4_perception_launch)
autoware_architect_build_deploy(${PROJECT_NAME} CameraLidarRadarCenterpointSerial.system dummy_modules;tier4_perception_launch)
autoware_architect_build_deploy(${PROJECT_NAME} CameraLidarRadarPointpaintingSerial.system dummy_modules;tier4_perception_launch)
autoware_architect_build_deploy(${PROJECT_NAME} LidarCenterpoint.system dummy_modules;tier4_perception_launch)
autoware_architect_build_deploy(${PROJECT_NAME} LidarTransfusion.system dummy_modules;tier4_perception_launch)

autoware_architect_build_deploy(${PROJECT_NAME} universe_perception_clr_centerpoint_serial.deployment dummy_modules;tier4_perception_launch)
