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
from ..models.links import ConnectionType

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


def _extract_external_interfaces(instance: Instance, interface_type: str) -> list:
    """Extract external interfaces from instance configuration."""
    interfaces = []
    
    if instance.configuration and instance.configuration.external_interfaces:
        ext_interfaces = instance.configuration.external_interfaces
        
        if interface_type in ext_interfaces:
            for interface in ext_interfaces[interface_type]:
                interfaces.append({
                    "name": interface.get("name", "")
                })
    
    return interfaces


def _generate_internal_interfaces(instance: Instance) -> list:
    """Generate internal interface mappings based on processed links."""
    internal_interfaces = []
    defined_interfaces = set()
    
    # Use the already processed links from LinkManager
    for link in instance.link_manager.get_all_links():
        from_port = link.from_port
        to_port = link.to_port
        
        # Use the stored connection type instead of re-determining it
        connection_type = link.connection_type
        
        # Handle external input to internal input
        if connection_type == ConnectionType.EXTERNAL_TO_INTERNAL:
            # Extract child instance name from to_port namespace
            child_instance_name = to_port.namespace[-1] if to_port.namespace else ""
            interface_key = f"{child_instance_name}/input/{to_port.name}"
            if interface_key not in defined_interfaces:
                internal_interfaces.append({
                    "name": interface_key,
                    "value": f"$(var input/{from_port.name})"
                })
                defined_interfaces.add(interface_key)
        
        # Handle internal output to external output
        elif connection_type == ConnectionType.INTERNAL_TO_EXTERNAL:
            # Extract child instance name from from_port namespace
            child_instance_name = from_port.namespace[-1] if from_port.namespace else ""
            interface_key = f"{child_instance_name}/output/{from_port.name}"
            if interface_key not in defined_interfaces:
                internal_interfaces.append({
                    "name": interface_key,
                    "value": f"$(var output/{to_port.name})"
                })
                defined_interfaces.add(interface_key)
        
        # Handle internal to internal connections
        elif connection_type == ConnectionType.INTERNAL_TO_INTERNAL:
            # Extract child instance names from port namespaces
            from_instance_name = from_port.namespace[-1] if from_port.namespace else ""
            to_instance_name = to_port.namespace[-1] if to_port.namespace else ""
            
            # Define the source output (if not already defined)
            source_interface_key = f"{from_instance_name}/output/{from_port.name}"
            if source_interface_key not in defined_interfaces:
                internal_interfaces.append({
                    "name": source_interface_key,
                    "value": f"$(var ns)/{from_instance_name}/{from_port.name}"
                })
                defined_interfaces.add(source_interface_key)
            
            # Define the destination input to reference the source
            dest_interface_key = f"{to_instance_name}/input/{to_port.name}"
            if dest_interface_key not in defined_interfaces:
                internal_interfaces.append({
                    "name": dest_interface_key,
                    "value": f"$(var {from_instance_name}/output/{from_port.name})"
                })
                defined_interfaces.add(dest_interface_key)
    
    return internal_interfaces


def _process_child_nodes(instance: Instance) -> list:
    """Process child nodes using instance hierarchy."""
    nodes = []
    
    for child_name, child_instance in instance.children.items():
        if child_instance.element_type == "module":
            package_name = _extract_package_name_from_module(child_instance)
            launcher_file = _get_launcher_filename(child_instance.configuration.name)
            
            # Generate arguments for this node based on configuration connections
            node_args = _generate_node_args(instance, child_name)
            
            nodes.append({
                "name": child_name,
                "package": package_name,
                "launcher_file": launcher_file,
                "args": node_args
            })
            
        elif child_instance.element_type == "pipeline":
            # Handle pipeline children
            child_launcher_file = _convert_to_snake_case(child_instance.name) + ".launch.xml"
            child_pipeline_path = f"{child_name}/{child_launcher_file}"
            
            # Generate arguments for this child pipeline based on configuration connections
            node_args = _generate_node_args(instance, child_name)
            
            nodes.append({
                "name": child_name,
                "package": None,  # Pipelines don't have packages
                "launcher_file": child_pipeline_path,
                "args": node_args,
                "is_pipeline": True  # Flag to distinguish from modules
            })
    
    return nodes


def _generate_node_args(instance: Instance, child_name: str) -> list:
    """Generate arguments for a child node based on connections."""
    node_args = []
    defined_args = set()
    
    for link in instance.link_manager.get_all_links():
        from_instance = link.from_port.namespace[-1] if link.from_port.namespace else ""
        to_instance = link.to_port.namespace[-1] if link.to_port.namespace else ""
        
        # Add output arg if child is source
        if from_instance == child_name:
            arg_name = f"output/{link.from_port.name}"
            if arg_name not in defined_args:
                node_args.append({"name": arg_name, "value": f"$(var {child_name}/output/{link.from_port.name})"})
                defined_args.add(arg_name)
        
        # Add input arg if child is destination
        if to_instance == child_name:
            arg_name = f"input/{link.to_port.name}"
            if arg_name not in defined_args:
                node_args.append({"name": arg_name, "value": f"$(var {child_name}/input/{link.to_port.name})"})
                defined_args.add(arg_name)
    
    return node_args


def _generate_launch_file(instance: Instance, output_dir: str, template_data: dict):
    """Generate the launch file using template."""
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


def _collect_component_interfaces(component: Instance, architecture_instance: Instance) -> dict:
    """Collect input and output interfaces for a component based on architecture-level connections."""
    inputs = []
    outputs = []
    
    # Use sets to track already added interfaces to avoid duplicates
    added_inputs = set()
    added_outputs = set()
    
    # Access the architecture's link manager to get the actual connections
    architecture_links = architecture_instance.link_manager.get_all_links()
    
    for link in architecture_links:
        from_port = link.from_port
        to_port = link.to_port
        connection_type = link.connection_type
        
        # Check if this link involves the current component
        component_involved = False
        
        # For external to internal connections
        if connection_type == ConnectionType.EXTERNAL_TO_INTERNAL:
            # Check if this component is the target
            if (len(to_port.namespace) >= 2 and 
                to_port.namespace[-1] == component.name):
                component_involved = True
                interface_key = to_port.name
                if interface_key not in added_inputs:
                    inputs.append({
                        "name": to_port.name,
                        "value": f"$(var input/{from_port.name})",
                        "msg_type": to_port.msg_type
                    })
                    added_inputs.add(interface_key)
        
        # For internal to external connections
        elif connection_type == ConnectionType.INTERNAL_TO_EXTERNAL:
            # Check if this component is the source
            if (len(from_port.namespace) >= 2 and 
                from_port.namespace[-1] == component.name):
                component_involved = True
                interface_key = from_port.name
                if interface_key not in added_outputs:
                    outputs.append({
                        "name": from_port.name,
                        "value": f"$(var output/{to_port.name})",
                        "msg_type": from_port.msg_type
                    })
                    added_outputs.add(interface_key)
        
        # For internal to internal connections
        elif connection_type == ConnectionType.INTERNAL_TO_INTERNAL:
            # Check if this component is the target (receives input)
            if (len(to_port.namespace) >= 2 and 
                to_port.namespace[-1] == component.name):
                source_component = from_port.namespace[-1] if from_port.namespace else ""
                interface_key = to_port.name
                if interface_key not in added_inputs:
                    inputs.append({
                        "name": to_port.name,
                        "value": f"$(var {source_component}/output/{from_port.name})",
                        "msg_type": to_port.msg_type
                    })
                    added_inputs.add(interface_key)
            
            # Check if this component is the source (produces output)
            if (len(from_port.namespace) >= 2 and 
                from_port.namespace[-1] == component.name):
                interface_key = from_port.name
                if interface_key not in added_outputs:
                    outputs.append({
                        "name": from_port.name,
                        "value": f"$(var ns)/{component.name}/{from_port.name}",
                        "msg_type": from_port.msg_type
                    })
                    added_outputs.add(interface_key)
    
    return {"inputs": inputs, "outputs": outputs}


def _collect_namespace_external_interfaces(components: list) -> dict:
    """Collect all external interfaces for a namespace from its components."""
    external_inputs = set()
    external_outputs = set()
    
    for component in components:
        if hasattr(component, 'link_manager'):
            for link in component.link_manager.get_all_links():
                from_port = link.from_port
                to_port = link.to_port
                connection_type = link.connection_type
                
                if connection_type == ConnectionType.EXTERNAL_TO_INTERNAL:
                    external_inputs.add(from_port.name)
                elif connection_type == ConnectionType.INTERNAL_TO_EXTERNAL:
                    external_outputs.add(to_port.name)
    
    return {
        "external_inputs": [{"name": name, "default_value": f"/default/{name}"} for name in sorted(external_inputs)],
        "external_outputs": [{"name": name, "default_value": f"/default/{name}"} for name in sorted(external_outputs)]
    }


def _generate_compute_unit_launcher(compute_unit: str, components: list, output_dir: str):
    """Generate compute unit launcher file that launches all namespaces in the compute unit."""
    # Compute unit launcher: output_dir/main_ecu/main_ecu.launch.xml
    compute_unit_dir = os.path.join(output_dir, compute_unit)
    os.makedirs(compute_unit_dir, exist_ok=True)
    
    launcher_file = os.path.join(compute_unit_dir, f"{compute_unit.lower()}.launch.xml")
    
    logger.debug(f"Creating compute unit launcher: {launcher_file}")
    
    # Group components by namespace to determine which namespace launchers to include
    namespace_groups = {}
    for component in components:
        namespace = component.namespace[0] if component.namespace else "default"
        if namespace not in namespace_groups:
            namespace_groups[namespace] = []
        namespace_groups[namespace].append(component)
    
    # Prepare template data
    namespaces_data = []
    for namespace, comps in sorted(namespace_groups.items()):
        # Collect external interfaces for this namespace
        external_interfaces = _collect_namespace_external_interfaces(comps)
        
        namespace_info = {
            "namespace": namespace,
            "component_count": len(comps),
            "args": []  # TODO: Add namespace-level arguments if needed
        }
        namespaces_data.append(namespace_info)
    
    template_data = {
        "compute_unit": compute_unit,
        "namespaces": namespaces_data
    }
    
    # Generate using template
    try:
        renderer = TemplateRenderer()
        launcher_xml = renderer.render_template('compute_unit_launcher.xml.jinja2', **template_data)
        
        with open(launcher_file, "w") as f:
            f.write(launcher_xml)
            
        logger.info(f"Generated compute unit launcher: {launcher_file}")
        
    except Exception as e:
        logger.error(f"Failed to generate compute unit launcher {launcher_file}: {e}")
        raise


def _generate_namespace_launcher(compute_unit: str, namespace: str, components: list, output_dir: str, architecture_instance: Instance):
    """Generate namespace launcher file that launches all components in the namespace."""
    # Namespace launcher: output_dir/main_ecu/perception/perception.launch.xml
    namespace_dir = os.path.join(output_dir, compute_unit, namespace)
    os.makedirs(namespace_dir, exist_ok=True)
    
    # Generate namespace-specific launcher filename
    launcher_file = os.path.join(namespace_dir, f"{namespace}.launch.xml")
    
    logger.debug(f"Creating namespace launcher: {launcher_file}")
    
    # Collect external interfaces for the namespace
    external_interfaces = _collect_namespace_external_interfaces(components)
    
    # Collect internal interface mappings from the architecture's link manager
    internal_interfaces = []
    defined_interfaces = set()
    
    # Use architecture-level links for internal interface mappings
    for link in architecture_instance.link_manager.get_all_links():
        from_port = link.from_port
        to_port = link.to_port
        connection_type = link.connection_type
        
        # Handle internal to internal connections within this namespace
        if connection_type == ConnectionType.INTERNAL_TO_INTERNAL:
            # Check if both ports are within the current namespace
            from_namespace = from_port.namespace[:-1] if len(from_port.namespace) > 1 else []
            to_namespace = to_port.namespace[:-1] if len(to_port.namespace) > 1 else []
            
            # Only include if both components are in the current namespace
            if (len(from_namespace) > 0 and from_namespace[0] == namespace and
                len(to_namespace) > 0 and to_namespace[0] == namespace):
                
                from_instance_name = from_port.namespace[-1] if from_port.namespace else ""
                to_instance_name = to_port.namespace[-1] if to_port.namespace else ""
                
                # Define the source output (if not already defined)
                source_interface_key = f"{from_instance_name}/output/{from_port.name}"
                if source_interface_key not in defined_interfaces:
                    internal_interfaces.append({
                        "name": source_interface_key,
                        "value": f"$(var ns)/{from_instance_name}/{from_port.name}"
                    })
                    defined_interfaces.add(source_interface_key)
                
                # Define the destination input to reference the source
                dest_interface_key = f"{to_instance_name}/input/{to_port.name}"
                if dest_interface_key not in defined_interfaces:
                    internal_interfaces.append({
                        "name": dest_interface_key,
                        "value": f"$(var {from_instance_name}/output/{from_port.name})"
                    })
                    defined_interfaces.add(dest_interface_key)
    
    # Prepare component data with detailed interface information
    components_data = []
    for component in components:
        # Get interfaces for this component using architecture-level connections
        component_interfaces = _collect_component_interfaces(component, architecture_instance)
        
        # Determine the component's namespace path for pipeline includes
        if len(component.namespace) > 1:
            # Remove the component name from namespace for path calculation
            component_namespace_path = "/".join(component.namespace[:-1])
        else:
            component_namespace_path = ""
        
        # Prepare component data
        component_data = {
            "name": component.name,
            "element_type": component.element_type,
            "inputs": component_interfaces["inputs"],
            "outputs": component_interfaces["outputs"],
            "args": [],  # Additional arguments can be added here
            "namespace_path": component_namespace_path
        }
        
        if component.element_type == "module":
            package_name = _extract_package_name_from_module(component)
            component_data.update({
                "package": package_name,
                "launcher_file": _get_launcher_filename(component.configuration.name) if component.configuration else f"{component.name}.launch.xml"
            })
        elif component.element_type == "pipeline":
            # For pipelines, construct the path correctly
            pipeline_path_parts = component.namespace[:-1]  # Exclude component name
            pipeline_path = "/".join(pipeline_path_parts) if pipeline_path_parts else component.name
            component_data.update({
                "namespace_path": pipeline_path,
                "launcher_file": _convert_to_snake_case(component.configuration.name) + ".launch.xml" if component.configuration else f"{component.name}.launch.xml"
            })
        
        components_data.append(component_data)
    
    # Prepare template data
    template_data = {
        "compute_unit": compute_unit,
        "namespace": namespace,
        "external_inputs": external_interfaces["external_inputs"],
        "external_outputs": external_interfaces["external_outputs"],
        "internal_interfaces": internal_interfaces,
        "components": components_data
    }
    
    # Generate using template
    try:
        renderer = TemplateRenderer()
        launcher_xml = renderer.render_template('namespace_launcher.xml.jinja2', **template_data)
        
        with open(launcher_file, "w") as f:
            f.write(launcher_xml)
            
        logger.info(f"Generated namespace launcher: {launcher_file}")
        
    except Exception as e:
        logger.error(f"Failed to generate namespace launcher {launcher_file}: {e}")
        raise


def generate_pipeline_launch_file(instance: Instance, output_dir: str):
    logger.debug(f"Generating launcher for {instance.name} (type: {instance.element_type}) in {output_dir}")
    
    if instance.element_type == "architecture":

        # parse children to extract compute units
        # collect components per compute unit
        # for each compute unit, create a directory and generate a launcher
        compute_unit_map = {}
        for child in instance.children.values():
            if child.compute_unit not in compute_unit_map:
                compute_unit_map[child.compute_unit] = []
            compute_unit_map[child.compute_unit].append(child)
        # Group components by compute unit and namespace
        namespace_map = {}
        for child in instance.children.values():
            key = (child.compute_unit, child.namespace[0] if child.namespace else "default")
            if key not in namespace_map:
                namespace_map[key] = []
            namespace_map[key].append(child)

        # Generate compute unit launcher
        # the compute unit launcher launches all components assigned to that compute unit
        # no arguments or interfaces are needed at this level
        # example: deployment/main_ecu/main_ecu.launch.xml
        for compute_unit, components in compute_unit_map.items():
            compute_unit_dir = os.path.join(output_dir, compute_unit)
            os.makedirs(compute_unit_dir, exist_ok=True)
            _generate_compute_unit_launcher(compute_unit, components, output_dir)

        # Generate namespace launcher
        # the interfaces this level are deterministic, already defined in the Instance input.
        # example: deployment/main_ecu/perception/perception.launch.xml
        for (compute_unit, namespace), components in namespace_map.items():
            _generate_namespace_launcher(compute_unit, namespace, components, output_dir, instance)

        # recursively call children pipelines/modules
        for child in instance.children.values():
            # Recursively call children pipelines/modules
            name_parts = child.name.split('/')
            path = os.path.join(output_dir,  child.compute_unit, *child.namespace)
            generate_pipeline_launch_file(child, path)
    elif instance.element_type == "pipeline":
        # Use existing configuration data with simplified extraction
        external_inputs = _extract_external_interfaces(instance, "input")
        external_outputs = _extract_external_interfaces(instance, "output")
        
        # Generate internal interface mappings using existing connection data
        internal_interfaces = _generate_internal_interfaces(instance)
        
        # Process child nodes using existing instance hierarchy
        nodes = _process_child_nodes(instance)
        
        # Prepare template data
        template_data = {
            "external_inputs": external_inputs,
            "external_outputs": external_outputs,
            "internal_interfaces": internal_interfaces,
            "nodes": nodes
        }
        
        # Generate launch file using template
        _generate_launch_file(instance, output_dir, template_data)

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
        raise ValueError(f"Invalid element type: {instance.element_type}")# example: deployment/main_ecu.launch.xml
