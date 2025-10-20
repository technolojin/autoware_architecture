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
import logging
from typing import List, Dict, Any
from .instances import Instance
from ..template_utils import TemplateRenderer

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


def _resolve_absolute_topic(port) -> str:
    """Resolve the absolute topic path from a port."""
    if hasattr(port, 'topic') and port.topic:
        if isinstance(port.topic, list):
            return "/" + "/".join(port.topic)
        else:
            return port.topic if port.topic.startswith('/') else '/' + port.topic
    
    # Fallback to constructing from namespace and name
    if hasattr(port, 'namespace') and hasattr(port, 'name'):
        if port.namespace:
            return "/" + "/".join(port.namespace + [port.name])
        else:
            return "/" + port.name
    
    # Last resort fallback
    return f"/default/{port.name}"


def _collect_all_modules_recursively(instance: Instance) -> List[Dict[str, Any]]:
    """Recursively collect all modules within a component, tracking their namespace paths."""
    modules = []
    
    def traverse(current_instance: Instance, pipeline_path: List[str]):
        """Recursively traverse instance tree to find all modules."""
        for child_name, child_instance in current_instance.children.items():
            if child_instance.element_type == "module":
                # Extract module information
                module_data = _extract_module_data(child_instance, pipeline_path)
                modules.append(module_data)
            elif child_instance.element_type == "pipeline":
                # For pipelines, add to the namespace path and continue traversing
                new_pipeline_path = pipeline_path + [child_name]
                traverse(child_instance, new_pipeline_path)
    
    # Start traversal from the root component (which could be a pipeline or module)
    if instance.element_type == "pipeline":
        traverse(instance, [])
    elif instance.element_type == "module":
        # If it's already a module at the top level, just extract it
        module_data = _extract_module_data(instance, [])
        modules.append(module_data)
    
    return modules


def _extract_module_data(module_instance: Instance, pipeline_path: List[str]) -> Dict[str, Any]:
    """Extract all necessary data from a module instance for launcher generation.
    
    This function uses the already-parsed parameters from parameter_manager instead of
    re-parsing the module configuration.
    """
    launch_config = module_instance.configuration.launch
    
    # Extract launch information
    package = launch_config.get("package", "")
    plugin = launch_config.get("plugin", "")
    executable = launch_config.get("executable", "")
    use_container = launch_config.get("use_container", False)  # Default to regular node
    container = launch_config.get("container_name", "perception_container")
    node_output = launch_config.get("node_output", "screen")
    
    # Calculate namespace groups for nested push-ros-namespace
    # pipeline_path contains the intermediate pipeline names
    namespace_groups = pipeline_path.copy()
    
    # Calculate full namespace path for documentation
    full_namespace_path = "/".join(pipeline_path) if pipeline_path else ""
    
    # Collect ports with resolved topics
    ports = []
    
    # Add input ports
    for port in module_instance.link_manager.get_all_in_ports():
        topic = _resolve_absolute_topic(port)
        ports.append({
            "direction": "input",
            "name": port.name,
            "topic": topic
        })
    
    # Add output ports
    for port in module_instance.link_manager.get_all_out_ports():
        topic = _resolve_absolute_topic(port)
        ports.append({
            "direction": "output",
            "name": port.name,
            "topic": topic
        })
    
    # Get parameter files and configurations from parameter_manager (already parsed and ordered)
    parameter_files = module_instance.parameter_manager.get_parameter_files_for_launch()
    configurations = module_instance.parameter_manager.get_configurations_for_launch()
    
    return {
        "name": module_instance.name,
        "package": package,
        "plugin": plugin,
        "executable": executable,
        "use_container": use_container,
        "container": container,
        "node_output": node_output,
        "namespace_groups": namespace_groups,
        "full_namespace_path": full_namespace_path,
        "ports": ports,
        "parameter_files": parameter_files,
        "configurations": configurations
    }


def _generate_compute_unit_launcher(compute_unit: str, components: list, output_dir: str):
    """Generate compute unit launcher file that launches all namespaces in the compute unit."""
    compute_unit_dir = os.path.join(output_dir, compute_unit)
    _ensure_directory(compute_unit_dir)
    
    launcher_file = os.path.join(compute_unit_dir, f"{compute_unit.lower()}.launch.xml")
    
    logger.debug(f"Creating compute unit launcher: {launcher_file}")
    
    # Group components by namespace to determine which namespace launchers to include
    namespace_groups = {}
    for component in components:
        namespace = component.namespace[0]
        if namespace not in namespace_groups:
            namespace_groups[namespace] = []
        namespace_groups[namespace].append(component)
    
    # Prepare template data
    namespaces_data = []
    for namespace, comps in sorted(namespace_groups.items()):
        namespace_info = {
            "namespace": namespace,
            "component_count": len(comps),
            "args": []
        }
        namespaces_data.append(namespace_info)
    
    template_data = {
        "compute_unit": compute_unit,
        "namespaces": namespaces_data
    }
    
    _render_template_to_file('compute_unit_launcher.xml.jinja2', launcher_file, template_data)


def _build_namespace_tree(modules: List[Dict[str, Any]], base_namespace: List[str]) -> Dict:
    """Build a tree structure from modules based on their namespace paths.
    
    Args:
        modules: List of module data dictionaries
        base_namespace: Base namespace from the component (e.g., ['perception', 'object_recognition'])
    
    Returns:
        A nested dictionary representing the namespace tree structure
    """
    tree = {}
    
    for module in modules:
        # Combine base namespace with module's namespace groups
        full_namespace = base_namespace + module['namespace_groups']
        
        # Add module to the leaf node
        if not full_namespace:
            # Module at root level
            if '__root__' not in tree:
                tree['__root__'] = {'modules': [], 'children': {}}
            tree['__root__']['modules'].append(module)
        else:
            # Navigate/create the tree structure and add module at the end
            current = tree
            for i, ns in enumerate(full_namespace):
                if ns not in current:
                    current[ns] = {'modules': [], 'children': {}}
                
                # If this is the last namespace in the path, add the module here
                if i == len(full_namespace) - 1:
                    current[ns]['modules'].append(module)
                else:
                    # Otherwise, continue navigating down
                    current = current[ns]['children']
    
    return tree


def _generate_component_launcher(compute_unit: str, namespace: str, components: list, output_dir: str):
    """Generate component launcher file that directly launches all modules in the component."""
    component_dir = os.path.join(output_dir, compute_unit, namespace)
    _ensure_directory(component_dir)
    
    launcher_file = os.path.join(component_dir, f"{namespace}.launch.xml")
    
    logger.debug(f"Creating component launcher: {launcher_file}")
    
    # Collect all modules recursively from all components in this namespace
    all_modules = []
    component_full_namespace = []
    for component in components:
        modules = _collect_all_modules_recursively(component)
        all_modules.extend(modules)
        # Extract the full namespace from the component (should be same for all components in this group)
        if not component_full_namespace and hasattr(component, 'namespace'):
            component_full_namespace = component.namespace.copy()
    
    # Build namespace tree
    namespace_tree = _build_namespace_tree(all_modules, component_full_namespace)
    
    # Prepare template data
    template_data = {
        "compute_unit": compute_unit,
        "namespace": namespace,
        "component_full_namespace": component_full_namespace,
        "namespace_tree": namespace_tree
    }
    
    _render_template_to_file('component_launcher.xml.jinja2', launcher_file, template_data)


def generate_pipeline_launch_file(instance: Instance, output_dir: str):
    """Main entry point for launcher generation."""
    logger.debug(f"Generating launcher for {instance.name} (type: {instance.element_type}) in {output_dir}")
    
    if instance.element_type == "architecture":
        # Group components by compute unit
        compute_unit_map = {}
        for child in instance.children.values():
            if child.compute_unit not in compute_unit_map:
                compute_unit_map[child.compute_unit] = []
            compute_unit_map[child.compute_unit].append(child)
        
        # Group components by compute unit and namespace
        namespace_map = {}
        for child in instance.children.values():
            key = (child.compute_unit, child.namespace[0])
            if key not in namespace_map:
                namespace_map[key] = []
            namespace_map[key].append(child)

        # Generate compute unit launchers
        for compute_unit, components in compute_unit_map.items():
            _generate_compute_unit_launcher(compute_unit, components, output_dir)

        # Generate component launchers (flattened, no pipeline launchers)
        for (compute_unit, namespace), components in namespace_map.items():
            _generate_component_launcher(compute_unit, namespace, components, output_dir)
    
    elif instance.element_type == "pipeline":
        # Pipelines are now handled within component launchers, no separate files needed
        logger.debug(f"Skipping separate pipeline launcher for {instance.name} - handled in component launcher")
        return
    
    elif instance.element_type == "module":
        # Module launch files are generated by the package build process
        logger.debug(f"Skipping module launcher for {instance.name} - handled by package")
        return
