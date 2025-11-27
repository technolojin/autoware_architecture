cmake_minimum_required(VERSION 3.14)
project(autoware_crosswalk_traffic_light_estimator)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library(crosswalk_traffic_light_estimator_cpp SHARED
  src/node.cpp
)
rclcpp_components_register_node(crosswalk_traffic_light_estimator_cpp
  PLUGIN "autoware::crosswalk_traffic_light_estimator::CrosswalkTrafficLightEstimatorNode"
  EXECUTABLE "crosswalk_traffic_light_estimator_node"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
