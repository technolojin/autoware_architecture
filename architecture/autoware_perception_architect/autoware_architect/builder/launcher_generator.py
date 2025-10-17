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
from ..models.parameters import ParameterType

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
    """Extract all necessary data from a module instance for launcher generation."""
    launch_config = module_instance.configuration.launch
    module_config = module_instance.configuration.config
    
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
    
    # Collect default parameter_files from module configuration
    default_param_files = {}
    if "parameter_files" in module_config:
        for param in module_config["parameter_files"]:
            param_name = param.get("name")
            param_default = param.get("default")
            if param_name and param_default:
                default_param_files[param_name] = param_default
    
    # Collect default configurations from module configuration
    default_configurations = {}
    if "configurations" in module_config:
        for config in module_config["configurations"]:
            config_name = config.get("name")
            config_default = config.get("default")
            if config_name:
                default_configurations[config_name] = config_default
    
    # Collect parameter overrides from parameter_set (via parameter_manager)
    parameter_set_params = {}
    parameter_set_configs = {}
    
    parameter_files = module_instance.parameter_manager.get_all_parameter_files()
    
    for param in parameter_files:
        if param.param_type == ParameterType.PARAMETER_FILES:
            parameter_set_params[param.name] = param.value
        elif param.param_type == ParameterType.CONFIGURATION:
            # Only include configurations that are not "none"
            if param.value != "none":
                parameter_set_configs[param.name] = param.value
    
    # Build final parameter_files list
    # Group into default files and override files for better organization
    default_param_file_list = []
    override_param_file_list = []
    
    for name, default_path in default_param_files.items():
        # Always add the default path first with package prefix
        # If the path already starts with $( or /, don't add prefix
        if default_path.startswith('$(') or default_path.startswith('/'):
            default_full_path = default_path
        else:
            default_full_path = f"$(find-pkg-share {package})/{default_path}"
        
        default_param_file_list.append({
            "path": default_full_path
        })
        
        # If parameter_set provides an override file AND it's different from default, add it to override list
        if name in parameter_set_params:
            override_path = parameter_set_params[name]
            # Only add override if it's actually different from the default
            if override_path != default_path and override_path != default_full_path:
                override_param_file_list.append({
                    "path": override_path
                })

    # Build final configurations list (defaults + parameter_set overrides)
    final_configurations = []
    
    # Merge default configurations with parameter_set overrides
    all_config_keys = set(default_configurations.keys()) | set(parameter_set_configs.keys())
    for config_name in sorted(all_config_keys):
        # Use parameter_set value if available, otherwise use default
        if config_name in parameter_set_configs:
            config_value = parameter_set_configs[config_name]
        else:
            config_value = default_configurations.get(config_name)
        
        # Only include if value is not None and not "none"
        if config_value is not None and config_value != "none":
            final_configurations.append({
                "name": config_name,
                "value": config_value
            })
    
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
        "default_param_files": default_param_file_list,
        "override_param_files": override_param_file_list,
        "configurations": final_configurations
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
