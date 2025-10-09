# Copyright 2025 TIER IV, inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import re
import logging
from .instances import Instance, DeploymentInstance
from ..template_utils import TemplateRenderer

logger = logging.getLogger(__name__)


def _convert_to_snake_case(name: str) -> str:
    """Convert CamelCase to snake_case."""
    # Handle the case where we have something like "CameraLidarFusion.pipeline"
    # First remove the .pipeline suffix if present
    if name.endswith('.pipeline'):
        name = name[:-9]  # Remove '.pipeline'
    
    # Convert CamelCase to snake_case
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()


def _get_launcher_filename(module_name: str) -> str:
    """Get launcher filename from module name."""
    # Remove .module suffix if present
    if module_name.endswith('.module'):
        module_name = module_name[:-7]  # Remove '.module'
    
    return _convert_to_snake_case(module_name) + ".launch.xml"


def _extract_package_name_from_module(instance: Instance) -> str:
    """Extract package name from module configuration."""
    if instance.element_type == "module" and instance.configuration:
        launch_config = instance.configuration.launch
        if launch_config and isinstance(launch_config, dict):
            return launch_config.get("package", "")
    return ""


def generate_pipeline_launch_file(instance: Instance, output_dir: str):
    logger.debug(f"Generating launcher for {instance.name} (type: {instance.element_type}) in {output_dir}")
    
    if instance.element_type == "architecture":
        # generate current architecture launch file
        # launcher per compute_unit
        # For architecture, we assume children are pipelines or modules under compute units
        # We will create a directory for each compute unit and generate launchers there
    

        # recursively call children pipelines
        for child in instance.children.values():
            # Split component name by slashes and use only the resulting parts as path
            name_parts = child.name.split('/')
            # Use the base namespace + split name parts
            base_namespace = child.namespace[:-1] if child.namespace else []  # Remove the component name from namespace
            path_parts = base_namespace + name_parts
            path = os.path.join(output_dir, *path_parts)
            generate_pipeline_launch_file(child, path)
    elif instance.element_type == "pipeline":
        # generate current pipeline launch file
        pipeline_launch_dict = {}
        pipeline_launch_dict["pipeline_name"] = instance.name
        
        # Extract external interfaces
        external_inputs = []
        external_outputs = []
        
        if instance.configuration and instance.configuration.external_interfaces:
            ext_interfaces = instance.configuration.external_interfaces
            
            # Process external inputs
            if "input" in ext_interfaces:
                for input_interface in ext_interfaces["input"]:
                    external_inputs.append({
                        "name": input_interface.get("name", "")
                    })
            
            # Process external outputs  
            if "output" in ext_interfaces:
                for output_interface in ext_interfaces["output"]:
                    external_outputs.append({
                        "name": output_interface.get("name", "")
                    })
        
        # Generate internal interface mappings based on connections
        internal_interfaces = []
        defined_interfaces = set()  # Track already defined interfaces to avoid duplicates
        
        # Create internal interface mappings based on the pipeline configuration connections
        if instance.configuration and instance.configuration.connections:
            for connection in instance.configuration.connections:
                from_part = connection.get("from", "")
                to_part = connection.get("to", "")
                
                # Parse the from and to parts
                if "." in from_part and "." in to_part:
                    from_parts = from_part.split(".")
                    to_parts = to_part.split(".")
                    
                    if len(from_parts) >= 2 and len(to_parts) >= 2:
                        from_instance = from_parts[0]
                        from_port = ".".join(from_parts[1:])
                        to_instance = to_parts[0]
                        to_port = ".".join(to_parts[1:])
                        
                        # Convert dots to slashes for proper ROS naming
                        from_port_ros = from_port.replace(".", "/")
                        to_port_ros = to_port.replace(".", "/")
                        
                        # Handle external input to internal input
                        if from_instance == "input":
                            interface_key = f"{to_instance}/{to_port_ros}"
                            if interface_key not in defined_interfaces:
                                internal_interfaces.append({
                                    "name": interface_key,
                                    "value": f"$(var input/{from_port_ros})"
                                })
                                defined_interfaces.add(interface_key)
                        
                        # Handle internal output to external output  
                        elif to_instance == "output":
                            interface_key = f"{from_instance}/{from_port_ros}"
                            if interface_key not in defined_interfaces:
                                internal_interfaces.append({
                                    "name": interface_key,
                                    "value": f"$(var output/{to_port_ros})"
                                })
                                defined_interfaces.add(interface_key)
                        
                        # Handle internal to internal connections
                        else:
                            # Define the source output (only if not already defined)
                            source_interface_key = f"{from_instance}/{from_port_ros}"
                            if source_interface_key not in defined_interfaces:
                                internal_interfaces.append({
                                    "name": source_interface_key,
                                    "value": f"$(var ns)/{from_instance}/objects"
                                })
                                defined_interfaces.add(source_interface_key)
                            
                            # Define the destination input to reference the source
                            dest_interface_key = f"{to_instance}/{to_port_ros}"
                            if dest_interface_key not in defined_interfaces:
                                internal_interfaces.append({
                                    "name": dest_interface_key,
                                    "value": f"$(var {from_instance}/{from_port_ros})"
                                })
                                defined_interfaces.add(dest_interface_key)
        
        # Process child nodes
        nodes = []
        for child_name, child_instance in instance.children.items():
            if child_instance.element_type == "module":
                package_name = _extract_package_name_from_module(child_instance)
                launcher_file = _get_launcher_filename(child_instance.configuration.name)
                
                # Generate arguments for this node based on configuration connections
                node_args = []
                
                # Find all connections involving this node
                if instance.configuration and instance.configuration.connections:
                    for connection in instance.configuration.connections:
                        from_part = connection.get("from", "")
                        to_part = connection.get("to", "")
                        
                        # Check if this connection involves the current child node
                        if "." in from_part:
                            from_parts = from_part.split(".")
                            if len(from_parts) >= 2 and from_parts[0] == child_name:
                                # This node is the source
                                port_path = ".".join(from_parts[1:])
                                # Convert dots to slashes for proper ROS naming
                                port_path_ros = port_path.replace(".", "/")
                                node_args.append({
                                    "name": port_path_ros,
                                    "value": f"$(var {child_name}/{port_path_ros})"
                                })
                        
                        if "." in to_part:
                            to_parts = to_part.split(".")
                            if len(to_parts) >= 2 and to_parts[0] == child_name:
                                # This node is the destination
                                port_path = ".".join(to_parts[1:])
                                # Convert dots to slashes for proper ROS naming
                                port_path_ros = port_path.replace(".", "/")
                                node_args.append({
                                    "name": port_path_ros,
                                    "value": f"$(var {child_name}/{port_path_ros})"
                                })
                
                nodes.append({
                    "name": child_name,
                    "package": package_name,
                    "launcher_file": launcher_file,
                    "args": node_args
                })
                
            elif child_instance.element_type == "pipeline":
                # Handle pipeline children
                child_launcher_file = _convert_to_snake_case(child_instance.name) + ".launch.xml"
                # Child pipelines are in subdirectories, so we need to construct the relative path
                child_pipeline_path = f"{child_name}/{child_launcher_file}"
                
                # Generate arguments for this child pipeline based on configuration connections
                node_args = []
                
                # Find all connections involving this child pipeline
                if instance.configuration and instance.configuration.connections:
                    for connection in instance.configuration.connections:
                        from_part = connection.get("from", "")
                        to_part = connection.get("to", "")
                        
                        # Check if this connection involves the current child pipeline
                        if "." in from_part:
                            from_parts = from_part.split(".")
                            if len(from_parts) >= 2 and from_parts[0] == child_name:
                                # This pipeline is the source
                                port_path = ".".join(from_parts[1:])
                                # Convert dots to slashes for proper ROS naming
                                port_path_ros = port_path.replace(".", "/")
                                node_args.append({
                                    "name": port_path_ros,
                                    "value": f"$(var {child_name}/{port_path_ros})"
                                })
                        
                        if "." in to_part:
                            to_parts = to_part.split(".")
                            if len(to_parts) >= 2 and to_parts[0] == child_name:
                                # This pipeline is the destination
                                port_path = ".".join(to_parts[1:])
                                # Convert dots to slashes for proper ROS naming
                                port_path_ros = port_path.replace(".", "/")
                                node_args.append({
                                    "name": port_path_ros,
                                    "value": f"$(var {child_name}/{port_path_ros})"
                                })
                
                nodes.append({
                    "name": child_name,
                    "package": None,  # Pipelines don't have packages
                    "launcher_file": child_pipeline_path,
                    "args": node_args,
                    "is_pipeline": True  # Flag to distinguish from modules
                })
        
        # Prepare template data
        template_data = {
            "external_inputs": external_inputs,
            "external_outputs": external_outputs,
            "internal_interfaces": internal_interfaces,
            "nodes": nodes
        }
        
        # Generate launch file using template
        os.makedirs(output_dir, exist_ok=True)
        launch_filename = _convert_to_snake_case(instance.name) + ".launch.xml"
        output_file_path = os.path.join(output_dir, launch_filename)
        
        logger.debug(f"Creating launcher file: {output_file_path}")
        
        # Ensure the directory exists for the output file
        os.makedirs(os.path.dirname(output_file_path), exist_ok=True)
        
        try:
            renderer = TemplateRenderer()
            launcher_xml = renderer.render_template('pipeline_launcher.xml.jinja2', **template_data)
            
            with open(output_file_path, "w") as f:
                f.write(launcher_xml)
                
            logger.info(f"Successfully generated launcher: {output_file_path}")
            
        except Exception as e:
            logger.error(f"Failed to generate launcher {output_file_path}: {e}")
            raise

        # recursively call children pipelines
        for child in instance.children.values():
            if child.element_type == "pipeline":
                # For pipeline children, simply use the child's name directly
                # The output_dir already contains the correct parent path
                child_name_parts = child.name.split('/')
                child_path = os.path.join(output_dir, *child_name_parts)
                generate_pipeline_launch_file(child, child_path)
    elif instance.element_type == "module":
        # module launch files are already generated on each package build process
        return
    else:
        raise ValueError(f"Invalid element type: {instance.element_type}")
