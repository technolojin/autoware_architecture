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


def _ensure_directory(directory_path: str) -> None:
    """Ensure directory exists by creating it if necessary."""
    os.makedirs(directory_path, exist_ok=True)


def _render_template_to_file(template_name: str, output_file_path: str, template_data: dict) -> None:
    """Render template and write to file with error handling."""
    try:
        renderer = TemplateRenderer()
        launcher_xml = renderer.render_template(template_name, **template_data)
        
        with open(output_file_path, "w") as f:
            f.write(launcher_xml)
            
        logger.info(f"Successfully generated launcher: {output_file_path}")
        
    except Exception as e:
        logger.error(f"Failed to generate launcher {output_file_path}: {e}")
        raise


def _process_links_with_interface_tracking(links, processor_func):
    """Process links with interface tracking to avoid duplicates."""
    interfaces = []
    defined_interfaces = set()
    
    for link in links:
        interface_items = processor_func(link, defined_interfaces)
        interfaces.extend(interface_items)
    
    return interfaces


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


def _process_link_for_internal_interfaces(link, defined_interfaces: set) -> list:
    """Process a single link for internal interface mapping."""
    interfaces = []
    from_port = link.from_port
    to_port = link.to_port
    connection_type = link.connection_type
    
    # Handle external input to internal input
    if connection_type == ConnectionType.EXTERNAL_TO_INTERNAL:
        child_instance_name = to_port.namespace[-1] if to_port.namespace else ""
        interface_key = f"{child_instance_name}/input/{to_port.name}"
        if interface_key not in defined_interfaces:
            interfaces.append({
                "name": interface_key,
                "value": f"$(var input/{from_port.name})"
            })
            defined_interfaces.add(interface_key)
    
    # Handle internal output to external output
    elif connection_type == ConnectionType.INTERNAL_TO_EXTERNAL:
        child_instance_name = from_port.namespace[-1] if from_port.namespace else ""
        interface_key = f"{child_instance_name}/output/{from_port.name}"
        if interface_key not in defined_interfaces:
            interfaces.append({
                "name": interface_key,
                "value": f"$(var output/{to_port.name})"
            })
            defined_interfaces.add(interface_key)
    
    # Handle internal to internal connections
    elif connection_type == ConnectionType.INTERNAL_TO_INTERNAL:
        from_instance_name = from_port.namespace[-1] if from_port.namespace else ""
        to_instance_name = to_port.namespace[-1] if to_port.namespace else ""
        
        # Define the source output (if not already defined)
        source_interface_key = f"{from_instance_name}/output/{from_port.name}"
        if source_interface_key not in defined_interfaces:
            interfaces.append({
                "name": source_interface_key,
                "value": f"$(var ns)/{from_instance_name}/{from_port.name}"
            })
            defined_interfaces.add(source_interface_key)
        
        # Define the destination input to reference the source
        dest_interface_key = f"{to_instance_name}/input/{to_port.name}"
        if dest_interface_key not in defined_interfaces:
            interfaces.append({
                "name": dest_interface_key,
                "value": f"$(var {from_instance_name}/output/{from_port.name})"
            })
            defined_interfaces.add(dest_interface_key)
    
    return interfaces


def _generate_internal_interfaces(instance: Instance) -> list:
    """Generate internal interface mappings based on processed links."""
    return _process_links_with_interface_tracking(
        instance.link_manager.get_all_links(),
        _process_link_for_internal_interfaces
    )


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
    _ensure_directory(output_dir)
    launch_filename = _convert_to_snake_case(instance.name) + ".launch.xml"
    output_file_path = os.path.join(output_dir, launch_filename)
    
    logger.debug(f"Creating launcher file: {output_file_path}")
    
    # Ensure the directory exists for the output file
    _ensure_directory(os.path.dirname(output_file_path))
    
    _render_template_to_file('pipeline_launcher.xml.jinja2', output_file_path, template_data)


def _process_link_for_component_interfaces(link, component_name: str, interface_data: dict) -> None:
    """Process a single link for component interface collection."""
    from_port = link.from_port
    to_port = link.to_port
    connection_type = link.connection_type
    
    # For external to internal connections
    if connection_type == ConnectionType.EXTERNAL_TO_INTERNAL:
        # Check if this component is the target
        if (len(to_port.namespace) >= 2 and 
            to_port.namespace[-1] == component_name):
            interface_key = to_port.name
            if interface_key not in interface_data["added_inputs"]:
                interface_data["inputs"].append({
                    "name": to_port.name,
                    "value": f"$(var input/{from_port.name})",
                    "msg_type": to_port.msg_type
                })
                interface_data["added_inputs"].add(interface_key)
    
    # For internal to external connections
    elif connection_type == ConnectionType.INTERNAL_TO_EXTERNAL:
        # Check if this component is the source
        if (len(from_port.namespace) >= 2 and 
            from_port.namespace[-1] == component_name):
            interface_key = from_port.name
            if interface_key not in interface_data["added_outputs"]:
                interface_data["outputs"].append({
                    "name": from_port.name,
                    "value": f"$(var output/{to_port.name})",
                    "msg_type": from_port.msg_type
                })
                interface_data["added_outputs"].add(interface_key)
    
    # For internal to internal connections
    elif connection_type == ConnectionType.INTERNAL_TO_INTERNAL:
        # Check if this component is the target (receives input)
        if (len(to_port.namespace) >= 2 and 
            to_port.namespace[-1] == component_name):
            source_component = from_port.namespace[-1] if from_port.namespace else ""
            interface_key = to_port.name
            if interface_key not in interface_data["added_inputs"]:
                interface_data["inputs"].append({
                    "name": to_port.name,
                    "value": f"$(var {source_component}/output/{from_port.name})",
                    "msg_type": to_port.msg_type
                })
                interface_data["added_inputs"].add(interface_key)
        
        # Check if this component is the source (produces output)
        if (len(from_port.namespace) >= 2 and 
            from_port.namespace[-1] == component_name):
            interface_key = from_port.name
            if interface_key not in interface_data["added_outputs"]:
                interface_data["outputs"].append({
                    "name": from_port.name,
                    "value": f"$(var ns)/{component_name}/{from_port.name}",
                    "msg_type": from_port.msg_type
                })
                interface_data["added_outputs"].add(interface_key)


def _collect_component_interfaces(component: Instance, architecture_instance: Instance) -> dict:
    """Collect input and output interfaces for a component based on architecture-level connections."""
    interface_data = {
        "inputs": [],
        "outputs": [],
        "added_inputs": set(),
        "added_outputs": set()
    }
    
    # Access the architecture's link manager to get the actual connections
    architecture_links = architecture_instance.link_manager.get_all_links()
    
    for link in architecture_links:
        _process_link_for_component_interfaces(link, component.name, interface_data)
    
    return {"inputs": interface_data["inputs"], "outputs": interface_data["outputs"]}


def _resolve_absolute_topic(port) -> str:
    """Resolve the absolute topic path from a port by traversing the reference chain."""
    
    # First check if the port itself has a topic set
    if hasattr(port, 'topic') and port.topic:
        return "/" + "/".join(port.topic)
    
    # For external ports, traverse the reference chain to find the actual topic
    if hasattr(port, 'reference') and port.reference:
        for ref_port in port.reference:
            if hasattr(ref_port, 'topic') and ref_port.topic:
                return "/" + "/".join(ref_port.topic)
            # Check if this reference port has its own references (deeper traversal)
            if hasattr(ref_port, 'reference') and ref_port.reference:
                for deeper_ref in ref_port.reference:
                    if hasattr(deeper_ref, 'topic') and deeper_ref.topic:
                        return "/" + "/".join(deeper_ref.topic)
    
    # For external inputs, check if connected to any internal ports with topics
    if hasattr(port, 'servers') and port.servers:
        for server_port in port.servers:
            if hasattr(server_port, 'topic') and server_port.topic:
                return "/" + "/".join(server_port.topic)
    
    # For external outputs, check if connected from any internal ports with topics  
    if hasattr(port, 'users') and port.users:
        for user_port in port.users:
            if hasattr(user_port, 'topic') and user_port.topic:
                return "/" + "/".join(user_port.topic)
    
    # If it's a global port, use the global topic
    if hasattr(port, 'is_global') and port.is_global and hasattr(port, 'topic') and port.topic:
        if isinstance(port.topic, list):
            return "/" + "/".join(port.topic)
        else:
            return port.topic if port.topic.startswith('/') else '/' + port.topic
    
    # Fallback to constructing from namespace and name
    if hasattr(port, 'namespace') and hasattr(port, 'name'):
        if hasattr(port, 'namespace') and port.namespace:
            return "/" + "/".join(port.namespace + [port.name])
        else:
            return "/" + port.name
    
    # Last resort fallback
    return f"/default/{port.name}"


def _process_link_for_external_interfaces(link, interface_dict: dict) -> None:
    """Process a single link for external interface collection."""
    from_port = link.from_port
    to_port = link.to_port
    connection_type = link.connection_type
    
    if connection_type == ConnectionType.EXTERNAL_TO_INTERNAL:
        # Resolve absolute topic from the reference chain
        absolute_topic = _resolve_absolute_topic(from_port)
        interface_dict["external_inputs"][from_port.name] = absolute_topic
    elif connection_type == ConnectionType.INTERNAL_TO_EXTERNAL:
        # Resolve absolute topic from the reference chain
        absolute_topic = _resolve_absolute_topic(to_port)
        interface_dict["external_outputs"][to_port.name] = absolute_topic


def _collect_namespace_external_interfaces(components: list) -> dict:
    """Collect all external interfaces for a namespace from its components."""
    interface_dict = {
        "external_inputs": {},
        "external_outputs": {}
    }
    
    for component in components:
        if hasattr(component, 'link_manager'):
            for link in component.link_manager.get_all_links():
                _process_link_for_external_interfaces(link, interface_dict)
    
    return {
        "external_inputs": [{"name": name, "default_value": topic} for name, topic in sorted(interface_dict["external_inputs"].items())],
        "external_outputs": [{"name": name, "default_value": topic} for name, topic in sorted(interface_dict["external_outputs"].items())]
    }


def _generate_compute_unit_launcher(compute_unit: str, components: list, output_dir: str):
    """Generate compute unit launcher file that launches all namespaces in the compute unit."""
    # Compute unit launcher: output_dir/main_ecu/main_ecu.launch.xml
    compute_unit_dir = os.path.join(output_dir, compute_unit)
    _ensure_directory(compute_unit_dir)
    
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
    
    _render_template_to_file('compute_unit_launcher.xml.jinja2', launcher_file, template_data)


def _process_link_for_namespace_interfaces(link, defined_interfaces: set, current_namespace: str) -> list:
    """Process a single link for namespace internal interface mapping."""
    interfaces = []
    from_port = link.from_port
    to_port = link.to_port
    connection_type = link.connection_type
    
    # Handle internal to internal connections within this namespace
    if connection_type == ConnectionType.INTERNAL_TO_INTERNAL:
        # Check if both ports are within the current namespace
        from_namespace = from_port.namespace[:-1] if len(from_port.namespace) > 1 else []
        to_namespace = to_port.namespace[:-1] if len(to_port.namespace) > 1 else []
        
        # Only include if both components are in the current namespace
        if (len(from_namespace) > 0 and from_namespace[0] == current_namespace and
            len(to_namespace) > 0 and to_namespace[0] == current_namespace):
            
            from_instance_name = from_port.namespace[-1] if from_port.namespace else ""
            to_instance_name = to_port.namespace[-1] if to_port.namespace else ""
            
            # Define the source output (if not already defined)
            source_interface_key = f"{from_instance_name}/output/{from_port.name}"
            if source_interface_key not in defined_interfaces:
                interfaces.append({
                    "name": source_interface_key,
                    "value": f"$(var ns)/{from_instance_name}/{from_port.name}"
                })
                defined_interfaces.add(source_interface_key)
            
            # Define the destination input to reference the source
            dest_interface_key = f"{to_instance_name}/input/{to_port.name}"
            if dest_interface_key not in defined_interfaces:
                interfaces.append({
                    "name": dest_interface_key,
                    "value": f"$(var {from_instance_name}/output/{from_port.name})"
                })
                defined_interfaces.add(dest_interface_key)
    
    return interfaces


def _collect_namespace_internal_interfaces(architecture_instance: Instance, namespace: str) -> list:
    """Collect internal interface mappings for a specific namespace."""
    def processor_func(link, defined_interfaces):
        return _process_link_for_namespace_interfaces(link, defined_interfaces, namespace)
    
    return _process_links_with_interface_tracking(
        architecture_instance.link_manager.get_all_links(),
        processor_func
    )


def _generate_namespace_launcher(compute_unit: str, namespace: str, components: list, output_dir: str, architecture_instance: Instance):
    """Generate namespace launcher file that launches all components in the namespace."""
    # Namespace launcher: output_dir/main_ecu/perception/perception.launch.xml
    namespace_dir = os.path.join(output_dir, compute_unit, namespace)
    _ensure_directory(namespace_dir)
    
    # Generate namespace-specific launcher filename
    launcher_file = os.path.join(namespace_dir, f"{namespace}.launch.xml")
    
    logger.debug(f"Creating namespace launcher: {launcher_file}")
    
    # Collect external interfaces for the namespace
    external_interfaces = _collect_namespace_external_interfaces(components)
    
    # Collect internal interface mappings from the architecture's link manager
    internal_interfaces = _collect_namespace_internal_interfaces(architecture_instance, namespace)
    
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
    
    _render_template_to_file('namespace_launcher.xml.jinja2', launcher_file, template_data)


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
            _ensure_directory(compute_unit_dir)
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
