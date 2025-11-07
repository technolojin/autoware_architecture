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
autoware_architect_build_deploy(${PROJECT_NAME} CameraLidarApollo.architecture dummy_modules)
autoware_architect_build_deploy(${PROJECT_NAME} CameraLidarCenterpoint.architecture dummy_modules)
autoware_architect_build_deploy(${PROJECT_NAME} CameraLidarPointpainting.architecture dummy_modules)
autoware_architect_build_deploy(${PROJECT_NAME} CameraLidarRadarCenterpointParallel.architecture dummy_modules)
autoware_architect_build_deploy(${PROJECT_NAME} CameraLidarRadarCenterpointSerial.architecture dummy_modules)
autoware_architect_build_deploy(${PROJECT_NAME} CameraLidarRadarPointpaintingSerial.architecture dummy_modules)
autoware_architect_build_deploy(${PROJECT_NAME} LidarCenterpoint.architecture dummy_modules)
autoware_architect_build_deploy(${PROJECT_NAME} LidarTransfusion.architecture dummy_modules)

autoware_architect_build_deploy(${PROJECT_NAME} universe_perception_clr_centerpoint_serial.deployment dummy_modules)
