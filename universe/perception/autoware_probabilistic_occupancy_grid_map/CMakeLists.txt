cmake_minimum_required(VERSION 3.14)
project(autoware_probabilistic_occupancy_grid_map)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library(pointcloud_based_occupancy_grid_map_cpp SHARED
  src/pointcloud_based_occupancy_grid_map_node.cpp
)
rclcpp_components_register_node(pointcloud_based_occupancy_grid_map_cpp
  PLUGIN "autoware::occupancy_grid_map::PointcloudBasedOccupancyGridMapNode"
  EXECUTABLE "pointcloud_based_occupancy_grid_map_node"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
