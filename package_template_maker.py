#!/usr/bin/env python3

import os
import sys
from pathlib import Path


def normalize_package_name(name: str) -> tuple[str, str]:
    """
    Normalize package name and determine if 'autoware_' prefix is needed.
    
    Returns:
        tuple: (full_package_name, class_name)
    """
    # Remove 'autoware_' prefix if it exists
    base_name = name.replace("autoware_", "")
    
    # Full package name with autoware_ prefix
    full_name = f"autoware_{base_name}" if not name.startswith("autoware_") else name
    
    # Convert snake_case to PascalCase for class name
    class_name = ''.join(word.capitalize() for word in base_name.split('_'))
    
    return full_name, class_name


def create_package_xml(package_name: str, author_name: str, author_email: str) -> str:
    """Create package.xml content."""
    return f'''<?xml version="1.0"?>
<package format="3">
  <name>{package_name}</name>
  <version>0.40.0</version>
  <description>The {package_name} package</description>

  <maintainer email="{author_email}">{author_name}</maintainer>
  <license>Apache License 2.0</license>
  <author email="{author_email}">{author_name}</author>

  <buildtool_depend>ament_cmake_auto</buildtool_depend>
  <buildtool_depend>autoware_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>autoware_architect</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
'''


def create_cmakelists(package_name: str, node_name: str, class_name: str) -> str:
    """Create CMakeLists.txt content."""
    return f'''cmake_minimum_required(VERSION 3.14)
project({package_name})

find_package(autoware_cmake REQUIRED)
autoware_package()

# Add component library
ament_auto_add_library({node_name}_cpp SHARED
  src/node.cpp
)
rclcpp_components_register_node({node_name}_cpp
  PLUGIN "autoware::{node_name}::{class_name}Node"
  EXECUTABLE "{node_name}_node"
)

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
  schema
)

autoware_architect_configure()

autoware_architect_parameter()

autoware_architect_generate_launcher()
'''


def create_module_yaml(class_name: str, package_name: str) -> str:
    """Create module.yaml content."""
    # Convert package name to node name (e.g., autoware_object_merger -> object_merger)
    node_name = package_name.replace("autoware_", "")
    
    return f'''# module information
name: {class_name}.module

launch:
  package: {package_name}
  plugin: autoware::{node_name}::{class_name}Node
  executable: {node_name}_node
  node_output: screen
  use_container: false
  container_name: pointcloud_container

# interfaces
inputs:
  - name: message_in
    message_type: autoware_perception_msgs/msg/DummyMessage

outputs:
  - name: message_out
    message_type: autoware_perception_msgs/msg/DummyMessage
    qos:
      reliability: reliable
      durability: transient_local

# configurations
parameter_files:
  - name: parameters
    default: config/{node_name}.param.yaml

configurations: []

# processes
processes:
  - name: dummy_process
    trigger_conditions:
      - and:
          - on_input: message_in
    outcomes:
      - to_output: message_out
'''


def create_schema_json(node_name: str, class_name: str) -> str:
    """Create schema.json content."""
    return f'''{{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Parameters for {class_name}",
  "type": "object",
  "definitions": {{
    "{node_name}": {{
      "type": "object",
      "properties": {{
        "number_1": {{
          "type": "integer",
          "description": "The maximum number of points that a cluster needs to contain in order to be considered valid.",
          "default": 1000
        }},
        "boolean_1": {{
          "type": "boolean",
          "description": "Whether to use point.z for clustering.",
          "default": false
        }}
      }},
      "required": ["number_1", "boolean_1"],
      "additionalProperties": false
    }}
  }},
  "properties": {{
    "/**": {{
      "type": "object",
      "properties": {{
        "ros__parameters": {{
          "$ref": "#/definitions/{node_name}"
        }}
      }},
      "required": ["ros__parameters"],
      "additionalProperties": false
    }}
  }},
  "required": ["/**"],
  "additionalProperties": false
}}
'''


def create_node_cpp(class_name: str, node_name: str) -> str:
    """Create C++ ROS2 node source file."""
    return f'''#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace autoware::{node_name}
{{
class {class_name}Node : public rclcpp::Node
{{
public:
  explicit {class_name}Node(const rclcpp::NodeOptions & options)
  : Node("{node_name}", options)
  {{
    // Create a timer that calls the callback every 2 seconds (1/2 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&{class_name}Node::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "{class_name}Node started");
  }}

private:
  void timer_callback()
  {{
    // Publish node namespace with node name
    std::string node_namespace = this->get_namespace();
    std::string node_name = this->get_name();
    RCLCPP_INFO(this->get_logger(), "Node %s %s", node_namespace.c_str(), node_name.c_str());
  }}

  rclcpp::TimerBase::SharedPtr timer_;
}};
}}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::{node_name}::{class_name}Node)
'''


def create_package_template(target_dir: str, package_name: str, 
                           author_name: str = "Taekjin Lee", 
                           author_email: str = "taekjin.lee@tier4.jp") -> bool:
    """
    Create a package template with the given name.
    
    Args:
        target_dir: Target directory where the package will be created
        package_name: Name of the package (with or without 'autoware_' prefix)
        author_name: Author name for package.xml
        author_email: Author email for package.xml
        
    Returns:
        bool: True if package was created, False if it already exists
    """
    # Normalize package name
    full_package_name, class_name = normalize_package_name(package_name)
    node_name = full_package_name.replace("autoware_", "")
    
    # Create package directory path
    package_dir = Path(target_dir) / full_package_name
    
    # Check if package already exists
    if package_dir.exists():
        print(f"⏭️  Package '{full_package_name}' already exists. Skipping...")
        return False
    
    print(f"📦 Creating package '{full_package_name}'...")
    
    # Create directory structure
    package_dir.mkdir(parents=True, exist_ok=True)
    (package_dir / "architecture").mkdir(exist_ok=True)
    (package_dir / "schema").mkdir(exist_ok=True)
    (package_dir / "src").mkdir(exist_ok=True)
    
    # Create package.xml
    package_xml_path = package_dir / "package.xml"
    package_xml_path.write_text(create_package_xml(full_package_name, author_name, author_email))
    print(f"  ✅ Created {package_xml_path.relative_to(Path(target_dir).parent)}")
    
    # Create CMakeLists.txt
    cmake_path = package_dir / "CMakeLists.txt"
    cmake_path.write_text(create_cmakelists(full_package_name, node_name, class_name))
    print(f"  ✅ Created {cmake_path.relative_to(Path(target_dir).parent)}")
    
    # Create node.cpp
    node_cpp_path = package_dir / "src" / "node.cpp"
    node_cpp_path.write_text(create_node_cpp(class_name, node_name))
    print(f"  ✅ Created {node_cpp_path.relative_to(Path(target_dir).parent)}")
    
    # Create module.yaml
    module_yaml_path = package_dir / "architecture" / f"{class_name}.module.yaml"
    module_yaml_path.write_text(create_module_yaml(class_name, full_package_name))
    print(f"  ✅ Created {module_yaml_path.relative_to(Path(target_dir).parent)}")
    
    # Create schema.json
    schema_json_path = package_dir / "schema" / f"{node_name}.schema.json"
    schema_json_path.write_text(create_schema_json(node_name, class_name))
    print(f"  ✅ Created {schema_json_path.relative_to(Path(target_dir).parent)}")
    
    print(f"✨ Package '{full_package_name}' created successfully!\n")
    return True


def main():
    """Main function to create multiple package templates."""
    # Configuration
    target_dir = "./universe/perception"
    package_names = [
        "autoware_tensorrt_yolox",
        "autoware_euclidean_cluster",
        "autoware_detected_object_feature_remover",
        "autoware_shape_estimation",
        "autoware_map_based_prediction",
        "autoware_lidar_centerpoint",
    ]
    
    # Optional: Override from command line arguments
    if len(sys.argv) > 1:
        target_dir = sys.argv[1]
    if len(sys.argv) > 2:
        package_names = sys.argv[2:]
    
    # Verify target directory exists
    target_path = Path(target_dir)
    if not target_path.exists():
        print(f"❌ Error: Target directory '{target_dir}' does not exist!")
        sys.exit(1)
    
    print(f"🚀 Starting package template creation in '{target_dir}'...\n")
    
    # Create packages
    created_count = 0
    skipped_count = 0
    
    for package_name in package_names:
        if create_package_template(target_dir, package_name):
            created_count += 1
        else:
            skipped_count += 1
    
    # Summary
    print("=" * 50)
    print(f"📊 Summary:")
    print(f"  ✅ Created: {created_count} package(s)")
    print(f"  ⏭️  Skipped: {skipped_count} package(s)")
    print(f"  📦 Total: {created_count + skipped_count} package(s)")
    print("=" * 50)


if __name__ == "__main__":
    main()