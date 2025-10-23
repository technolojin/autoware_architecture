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

if TYPE_CHECKING:
    from .instances import Instance

logger = logging.getLogger(__name__)


class ParameterTemplateGenerator:
    """Generates parameter set templates for deployment instances.
    
    This class handles:
    1. Collecting module parameter data from deployment instance tree
    2. Creating namespace-based directory structures
    3. Copying and organizing parameter configuration files
    4. Generating parameter set template files from templates
    """
    
    def __init__(self, root_instance: 'Instance'):
        """Initialize the parameter template generator.
        
        Args:
            root_instance: The root deployment instance to generate templates for
        """
        self.root_instance = root_instance
    
    # =========================================================================
    # Public API Methods
    # =========================================================================
    
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
    
    def collect_module_parameter_files_for_template(self, base_namespace: str = "") -> List[Dict[str, Any]]:
        """Collect parameter template data for all modules in the deployment instance.
        
        Args:
            base_namespace: Base namespace to prepend to module namespaces
            
        Returns:
            List of dictionaries containing module parameter template data
        """
        module_data = []
        self._collect_module_parameter_files_recursive(self.root_instance, module_data, base_namespace)
        return module_data
    
    # =========================================================================
    # Helper Methods for Template Generation
    # =========================================================================
    
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
            # Use the instance's namespace_str directly
            full_namespace = instance.namespace_str
            
            # Get parameter information from parameter_manager
            parameter_files, configurations = self._extract_parameters_from_manager(instance)
            
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
    
    def _extract_parameters_from_manager(self, module_instance: 'Instance') -> tuple[Dict[str, str], List[Dict[str, Any]]]:
        """Extract parameter files and configurations from parameter manager.
        
        Args:
            module_instance: Module instance to extract parameters from
            
        Returns:
            Tuple of (parameter_files_dict, configurations_list)
        """
        parameter_files = {}
        configurations = []
        
        # Get all parameters from the parameter manager
        all_parameters = module_instance.parameter_manager.get_all_parameter_files()
        
        # Use the instance's namespace_str directly for parameter file paths
        base_path = module_instance.namespace_str
        
        for param in all_parameters:
            param_name = param.name
            param_type = param.param_type
            
            if param_type.value == "parameter":  # ParameterType.PARAMETER_FILES
                # Generate template path based on namespace and parameter name
                template_path = f"{base_path}/{param_name}.param.yaml"
                parameter_files[param_name] = template_path
            
            elif param_type.value == "configuration":  # ParameterType.CONFIGURATION
                configuration = {
                    "name": param_name,
                    "type": param.data_type,
                    "value": param.value
                }
                configurations.append(configuration)
        
        return parameter_files, configurations
    
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
            variable_path = "$(var config_path)" + "/" + relative_path.replace("\\", "/")
            updated_parameter_files[param_name] = variable_path
        
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
        # Use ROS 2 API to find the install directory for the package
        try:
            import ament_index_python.packages
            package_name = template_path.strip("/").split("/")[0]
            install_dir = ament_index_python.packages.get_package_share_directory(package_name)
        except Exception as e:
            logger.warning(f"Could not find install directory for package '{package_name}': {e}")
            return None
        
        # Search for .param.yaml files matching the parameter name
        if os.path.exists(install_dir):
            for root, dirs, files in os.walk(install_dir):
                for file in files:
                    if file == f"{param_name}.param.yaml":
                        return os.path.join(root, file)

        return None
