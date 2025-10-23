cmake_minimum_required(VERSION 3.14)
project(autoware_map_based_prediction)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library(map_based_prediction_cpp SHARED
  src/node.cpp
)
rclcpp_components_register_node(map_based_prediction_cpp
  PLUGIN "autoware::map_based_prediction::MapBasedPredictionNode"
  EXECUTABLE "map_based_prediction"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
