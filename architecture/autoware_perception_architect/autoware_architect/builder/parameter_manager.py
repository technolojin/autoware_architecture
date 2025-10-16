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

import logging
from typing import TYPE_CHECKING, List, Dict, Any, Optional
import os
import shutil

from ..models.parameters import ParameterList, ParameterType

if TYPE_CHECKING:
    from .instances import Instance

logger = logging.getLogger(__name__)


class ParameterManager:
    """Manages parameter operations for Instance objects."""
    
    def __init__(self, instance: 'Instance'):
        self.instance = instance
        self.parameter_files: ParameterList = ParameterList()

    def set_parameter_file(self, param_file):
        """Set parameter based on instance element type."""
        # in case of pipeline, search parameter connection and call set_parameter for children
        if self.instance.element_type == "pipeline":
            self._set_pipeline_parameter_file(param_file)
        # in case of module, set the parameter
        elif self.instance.element_type == "module":
            self._set_module_parameter_file(param_file)
        else:
            raise ValueError(f"Invalid element type: {self.instance.element_type}")

    def _set_pipeline_parameter_file(self, param_file):
        """Set parameter for pipeline element type."""
        if self.instance.element_type != "pipeline":
            raise ValueError("_set_pipeline_parameter is only supported for pipeline")
        param_name = param_file.get("name")

        # check external_interfaces/parameter
        cfg_pipeline_parameter_list = self.instance.configuration.external_interfaces.get("parameter")
        # check if the param_list_yaml is superset of pipeline_parameter_list
        cfg_pipeline_parameter_list = [param.get("name") for param in cfg_pipeline_parameter_list]
        if param_name not in cfg_pipeline_parameter_list:
            raise ValueError(f"Parameter not found: '{param_name}' in {cfg_pipeline_parameter_list}")

        # check parameter_files to connect parameters to the children
        cfg_param_connection_list = self.instance.configuration.parameter_files
        for cfg_connection in cfg_param_connection_list:
            param_from = cfg_connection.get("from")
            param_from_name = param_from.split(".")[1]
            if param_from_name != param_name:
                continue

            param_to = cfg_connection.get("to")
            param_to_inst_name = param_to.split(".")[0]
            child_instance = self.instance.get_child(param_to_inst_name)

            # set the parameter to the child instance
            if child_instance.element_type == "pipeline":
                param_file["name"] = param_to.split(".")[2]

            child_instance.parameter_manager.set_parameter_file(param_file)

    def _set_module_parameter_file(self, param_file):
        """Set parameter for module element type."""
        if self.instance.element_type != "module":
            raise ValueError("_set_module_parameter_file is only supported for module")
        param_path_list = param_file.get("parameter_files")
        if not param_path_list:
            raise ValueError(f"No parameter paths found in parameter: {param_file}")
        # get list of parameter paths, which comes in dictionary format
        for param_path in param_path_list:
            param_keys = param_path.keys()
            for param_key in param_keys:
                param_value = param_path.get(param_key)
                # Set parameter with metadata indicating it's a template path
                self.parameter_files.set_parameter(
                    param_key, 
                    param_value, 
                    param_type=ParameterType.PARAMETER_FILES,
                    data_type="path",
                    allow_substs=True
                )
        for param in self.parameter_files.list:
            logger.debug(f"  Parameter: {param.name} = {param.value}")

    def initialize_module_parameters(self):
        """Initialize parameters for module element during module configuration."""
        if self.instance.element_type != "module":
            return
            
        # set parameter_files
        for cfg_param in self.instance.configuration.parameter_files:
            param_name = cfg_param.get("name")
            param_value = cfg_param.get("default")
            param_schema = cfg_param.get("schema")
            self.parameter_files.set_parameter(
                param_name, 
                param_value,
                param_type=ParameterType.PARAMETER_FILES,
                data_type="string",
                schema_path=param_schema,
                allow_substs=cfg_param.get("allow_substs", True)
            )

    def get_parameter(self, parameter_name: str):
        """Get parameter value by name."""
        return self.parameter_files.get_parameter(parameter_name)

    def get_all_parameter_files(self):
        """Get all parameter_files."""
        return self.parameter_files.list
    
    def collect_module_parameter_files_for_template(self, base_namespace: str = "") -> List[Dict[str, Any]]:
        """Collect parameter template data for all modules in the deployment instance.
        
        Args:
            base_namespace: Base namespace to prepend to module namespaces
            
        Returns:
            List of dictionaries containing module parameter template data
        """
        module_data = []
        self._collect_module_parameter_files_recursive(self.instance, module_data, base_namespace)
        return module_data
    
    def _collect_module_parameter_files_recursive(self, instance: 'Instance', 
                                           module_data: List[Dict[str, Any]], 
                                           current_namespace: str = "") -> None:
        """Recursively collect module parameter_files with their full namespaces.
        
        Args:
            instance: Current instance to process
            module_data: List to append module parameter data to
            current_namespace: Current namespace path
        """
        if instance.element_type == "module":
            # Use the instance's namespace_str directly - it already contains the correct namespace
            full_namespace = instance.namespace_str
            
            # Extract parameter information from module configuration
            parameter_files, configurations = self._extract_module_parameter_info(instance)
            
            if parameter_files or configurations:
                module_info = {
                    "node": full_namespace,
                    "parameter_files": parameter_files,
                    "configurations": configurations
                }
                module_data.append(module_info)
        
        # Recursively process children
        if hasattr(instance, 'children') and instance.children:
            for child in instance.children.values():
                self._collect_module_parameter_files_recursive(child, module_data, current_namespace)
    
    def _extract_module_parameter_info(self, module_instance: 'Instance') -> tuple[Dict[str, str], List[Dict[str, Any]]]:
        """Extract parameter paths and configurations from module configuration.
        
        Args:
            module_instance: Module instance to extract parameter_files from
            
        Returns:
            Tuple of (parameter_files_dict, configurations_list)
        """
        parameter_files = {}
        configurations = []
        
        if (hasattr(module_instance, 'configuration') and 
            hasattr(module_instance.configuration, 'parameter_files') and 
            module_instance.configuration.parameter_files):
            
            # Use the instance's namespace_str directly for parameter file paths
            base_path = module_instance.namespace_str
            
            for param in module_instance.configuration.parameter_files:
                param_name = param.get('name')
                if param_name:
                    # Generate template path based on namespace and parameter name
                    template_path = f"{base_path}/{param_name}.param.yaml"
                    parameter_files[param_name] = template_path
        
        # Extract configurations from module configuration
        if (hasattr(module_instance, 'configuration') and 
            hasattr(module_instance.configuration, 'configurations') and 
            module_instance.configuration.configurations):
            
            for config in module_instance.configuration.configurations:
                config_name = config.get('name')
                config_type = config.get('type', 'string')
                config_value = config.get('value')
                
                if config_name is not None:
                    configuration = {
                        "name": config_name,
                        "type": config_type,
                        "value": config_value
                    }
                    configurations.append(configuration)
        
        # Get any runtime parameter overrides from the parameter manager
        for param in module_instance.parameter_manager.get_all_parameter_files():
            if hasattr(param, 'param_type') and param.param_type == ParameterType.CONFIGURATION:
                # This is a runtime configuration override
                param_override = {
                    "name": param.name,
                    "type": getattr(param, 'data_type', 'string'),
                    "value": param.value
                }
                configurations.append(param_override)
        
        return parameter_files, configurations
    
    def generate_parameter_set_template(self, deployment_name: str, 
                                      template_renderer, output_dir: str) -> str:
        """Generate parameter set template for the entire deployment.
        
        Args:
            deployment_name: Name of the deployment
            template_renderer: Template renderer instance
            output_dir: Output directory for the template file
            
        Returns:
            Path to the generated template file
        """
        # Create parameter set directory structure
        parameter_set_root = os.path.join(output_dir, f"{deployment_name}.parameter_set")
        os.makedirs(parameter_set_root, exist_ok=True)
        
        # Collect all module parameter data
        module_data = self.collect_module_parameter_files_for_template()
        
        # Copy config files to namespace structure and update paths
        for module in module_data:
            self._create_namespace_structure_and_copy_configs(module, parameter_set_root)
        
        # Prepare template data
        template_data = {
            "name": f"{deployment_name}.parameter_set",
            "parameters": module_data
        }
        
        # Generate output filename
        output_filename = f"{deployment_name}.parameter_set.yaml"
        output_path = os.path.join(output_dir, output_filename)
        
        # Render template
        template_renderer.render_template_to_file(
            "parameter_set.yaml.jinja2", 
            output_path, 
            **template_data
        )
        
        logger.info(f"Generated parameter set template: {output_path}")
        logger.info(f"Generated parameter set structure at: {parameter_set_root}")
        return output_path
    
    def _create_namespace_structure_and_copy_configs(self, module_data: Dict[str, Any], 
                                                   parameter_set_root: str) -> None:
        """Create namespace folder structure and copy config files.
        
        Args:
            module_data: Module data containing node path and parameter_files
            parameter_set_root: Root directory for parameter set
        """
        node_path = module_data["node"]
        parameter_files = module_data["parameter_files"]
        
        # Create namespace directory structure
        # node_path is like "/perception/object_recognition/detection/centerpoint"
        namespace_dir = os.path.join(parameter_set_root, node_path.lstrip('/'))
        os.makedirs(namespace_dir, exist_ok=True)
        
        # Copy parameter files and update paths in module_data
        updated_parameter_files = {}
        for param_name, original_path in parameter_files.items():
            # Find the source config file
            source_config_path = self._find_source_config_file(param_name, original_path)
            
            # Copy config file to namespace directory
            dest_filename = f"{param_name}.param.yaml"
            dest_path = os.path.join(namespace_dir, dest_filename)
            
            if source_config_path and os.path.exists(source_config_path):
                try:
                    shutil.copy2(source_config_path, dest_path)
                    logger.debug(f"Copied {source_config_path} to {dest_path}")
                except Exception as e:
                    logger.warning(f"Failed to copy {source_config_path} to {dest_path}: {e}")
                    # Create empty config file if copy fails
                    self._create_empty_config_file(dest_path, param_name)
            else:
                logger.warning(f"Source config file not found for {param_name}: {original_path}")
                # Create empty config file when source is not found
                self._create_empty_config_file(dest_path, param_name)
            
            # Update path to be relative to parameter set root
            relative_path = os.path.relpath(dest_path, parameter_set_root)
            updated_parameter_files[param_name] = relative_path
        
        # Update module data with new paths
        module_data["parameter_files"] = updated_parameter_files
    
    def _create_empty_config_file(self, dest_path: str, param_name: str) -> None:
        """Create an empty config file with basic ROS parameter structure.
        
        Args:
            dest_path: Destination path for the config file
            param_name: Parameter name for logging
        """
        try:
            # Create basic ROS parameter file structure
            empty_config_content = """/**:
  ros__parameters:
    # TODO: Add parameters for {}
""".format(param_name)
            
            with open(dest_path, 'w') as f:
                f.write(empty_config_content)
            
            logger.info(f"Created empty config file: {dest_path}")
            
        except Exception as e:
            logger.error(f"Failed to create empty config file {dest_path}: {e}")
    
    def _find_source_config_file(self, param_name: str, template_path: str) -> Optional[str]:
        """Find the actual source config file generated from schema.
        
        Args:
            param_name: Parameter name
            template_path: Template path from module configuration
            
        Returns:
            Path to the actual config file, or None if not found
        """
        # Look for config files in the install directory
        # The config files are generated by the parameter_process.py script
        # Use relative path resolution to find workspace root dynamically
        current_file_dir = os.path.dirname(os.path.abspath(__file__))
        workspace_root = os.path.abspath(os.path.join(current_file_dir, "../../../../../.."))
        install_dir = os.path.join(workspace_root, "install")
        
        # Search for .param.yaml files matching the parameter name
        if os.path.exists(install_dir):
            for root, dirs, files in os.walk(install_dir):
                for file in files:
                    if file == f"{param_name}.param.yaml":
                        return os.path.join(root, file)
        
        # If not found in install directory, try to construct from template_path
        if template_path and not template_path.startswith('/'):
            # Relative path - try to find in various locations
            possible_paths = [
                os.path.join(workspace_root, template_path),
                os.path.join(workspace_root, "src", template_path),
                os.path.join(install_dir, template_path)
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    return path
        
        return None
