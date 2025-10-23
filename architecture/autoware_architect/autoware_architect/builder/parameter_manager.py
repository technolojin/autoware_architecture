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

from ..models.parameters import ParameterList, ParameterType, Parameter

if TYPE_CHECKING:
    from .instances import Instance

logger = logging.getLogger(__name__)


class ParameterManager:
    """Manages parameter operations for Instance objects.
    
    This class handles:
    1. Applying parameters from parameter sets to target nodes
    2. Initializing module parameters from configuration
    3. Managing parameter values and configurations
    4. Resolving parameter file paths with package prefixes
    """
    
    def __init__(self, instance: 'Instance'):
        self.instance = instance
        self.parameter_files: ParameterList = ParameterList()

    # =========================================================================
    # Public API Methods
    # =========================================================================
    
    def get_parameter(self, parameter_name: str):
        """Get parameter value by name."""
        return self.parameter_files.get_parameter(parameter_name)

    def get_all_parameter_files(self):
        """Get all parameter_files."""
        return self.parameter_files.list
    
    def get_parameter_files_for_launch(self) -> List[Dict[str, str]]:
        """Get parameter files formatted for launcher generation with resolved paths.
        
        Returns list of dicts with 'path' key, in order (defaults first, then overrides).
        """
        package_name = None
        if self.instance.element_type == "module" and self.instance.configuration:
            launch_config = self.instance.configuration.launch
            package_name = launch_config.get("package")
        
        result = []
        for param in self.parameter_files.get_parameter_files_ordered():
            resolved_path = self._resolve_parameter_file_path(param.value, package_name, param.is_default)
            result.append({"path": resolved_path})
        
        return result
    
    def get_configurations_for_launch(self) -> List[Dict[str, Any]]:
        """Get configurations formatted for launcher generation.
        
        Returns list of dicts with 'name' and 'value' keys, filtering out 'none' values.
        """
        result = []
        for param in self.parameter_files.get_configurations():
            # Only include if value is not None and not "none"
            if param.value is not None and param.value != "none":
                result.append({
                    "name": param.name,
                    "value": param.value
                })
        return result

    # =========================================================================
    # Parameter Path Resolution
    # =========================================================================
    
    def _resolve_parameter_file_path(self, path: str, package_name: Optional[str] = None, 
                                     is_default: bool = False) -> str:
        """Resolve parameter file path with package prefix if needed.
        
        Args:
            path: The parameter file path
            package_name: The ROS package name for default parameters
            is_default: Whether this is a default parameter file
            
        Returns:
            Resolved path with package prefix if applicable
        """
        # If path already starts with $( or /, don't add prefix
        if path.startswith('$(') or path.startswith('/'):
            return path
        
        # For default parameters, add package prefix if package name is available
        if is_default and package_name:
            return f"$(find-pkg-share {package_name})/{path}"
        
        # For overrides or when no package name, return as-is
        return path

    # =========================================================================
    # Parameter Application (from parameter sets)
    # =========================================================================
    
    def apply_node_parameters(self, node_namespace: str, parameter_files: list, configurations: list):
        """Apply parameters directly to a target node using new parameter set format.
        
        This method finds a node by its absolute namespace and applies both parameter_files 
        and configurations directly to it. Configurations will override parameter_files.
        
        Args:
            node_namespace: Absolute namespace path to the target node 
                           (e.g., "/perception/object_recognition/node_tracker")
            parameter_files: List of parameter file mappings 
                            (e.g., [{"model_param_path": "path/to/file.yaml"}])
            configurations: List of direct parameter configurations 
                           (e.g., [{"name": "build_only", "type": "bool", "value": false}])
        """
        target_instance = self._find_node_by_namespace(node_namespace)
        if target_instance is None:
            logger.warning(f"Target node not found: {node_namespace}")
            return
            
        if target_instance.element_type != "module":
            logger.warning(f"Target node is not a module: {node_namespace} (type: {target_instance.element_type})")
            return
        
        logger.info(f"Applying parameters to node: {node_namespace}")
            
        # Apply parameter files first (as overrides, not defaults)
        if parameter_files:
            for param_file_mapping in parameter_files:
                for param_name, param_path in param_file_mapping.items():
                    target_instance.parameter_manager.parameter_files.set_parameter(
                        param_name,
                        param_path,
                        param_type=ParameterType.PARAMETER_FILES,
                        data_type="path",
                        allow_substs=True,
                        is_default=False  # Parameter set overrides are not defaults
                    )
        
        # Apply configurations (these override parameter files)
        if configurations:
            for config in configurations:
                config_name = config.get("name")
                config_type = config.get("type", "string")
                config_value = config.get("value")
                
                target_instance.parameter_manager.parameter_files.set_parameter(
                    config_name,
                    config_value,
                    param_type=ParameterType.CONFIGURATION,
                    data_type=config_type,
                    allow_substs=True,
                    is_default=False  # Parameter set configurations are not defaults
                )

    # =========================================================================
    # Node Finding (helper methods for parameter application)
    # =========================================================================
    
    def _find_node_by_namespace(self, target_namespace: str):
        """Find a node instance by its absolute namespace path.
        
        Args:
            target_namespace: Absolute namespace path (e.g., "/perception/object_recognition/node_tracker")
            
        Returns:
            Instance object if found, None otherwise
        """
        # Start from the root deployment instance
        current_instance = self.instance
        
        # Navigate to the root deployment instance
        while current_instance.parent is not None:
            current_instance = current_instance.parent
            
        # If we're at the deployment root (element_type == "architecture"),
        # search in its children directly since parameter_set paths don't include the deployment name
        if current_instance.element_type == "architecture":
            for child in current_instance.children.values():
                result = self._traverse_to_namespace(child, target_namespace)
                if result is not None:
                    return result
            return None
        
        # Otherwise, traverse down from current instance
        return self._traverse_to_namespace(current_instance, target_namespace)
    
    def _traverse_to_namespace(self, instance, target_namespace: str):
        """Recursively traverse instance tree to find target namespace.
        
        Args:
            instance: Current instance to check
            target_namespace: Target namespace to find
            
        Returns:
            Instance object if found, None otherwise
        """
        # Check if current instance matches the target namespace
        if instance.namespace_str == target_namespace:
            return instance
            
        # Check if target namespace starts with current instance namespace
        # This means we need to look deeper in this branch
        if target_namespace.startswith(instance.namespace_str + "/") or target_namespace == instance.namespace_str:
            # Search in children
            for child in instance.children.values():
                result = self._traverse_to_namespace(child, target_namespace)
                if result is not None:
                    return result
        
        return None

    # =========================================================================
    # Module Parameter Initialization
    # =========================================================================
    
    def initialize_module_parameters(self):
        """Initialize parameters for module element during module configuration.
        
        This method initializes both default parameter_files and default configurations
        from the module's configuration file.
        """
        if self.instance.element_type != "module":
            return
        
        # 1. Set default parameter_files from module configuration
        if hasattr(self.instance.configuration, 'parameter_files') and self.instance.configuration.parameter_files:
            for cfg_param in self.instance.configuration.parameter_files:
                param_name = cfg_param.get("name")
                param_value = cfg_param.get("value", cfg_param.get("default"))
                param_schema = cfg_param.get("schema")
                self.parameter_files.set_parameter(
                    param_name, 
                    param_value,
                    param_type=ParameterType.PARAMETER_FILES,
                    data_type="string",
                    schema_path=param_schema,
                    allow_substs=cfg_param.get("allow_substs", True),
                    is_default=True  # These are default parameter files
                )
        
        # 2. Set default configurations from module configuration
        if hasattr(self.instance.configuration, 'configurations') and self.instance.configuration.configurations:
            for cfg_config in self.instance.configuration.configurations:
                config_name = cfg_config.get("name")
                config_value = cfg_config.get("value", cfg_config.get("default"))
                config_type = cfg_config.get("type", "string")
                
                # Only set if a default value is provided
                if config_value is not None:
                    self.parameter_files.set_parameter(
                        config_name,
                        config_value,
                        param_type=ParameterType.CONFIGURATION,
                        data_type=config_type,
                        allow_substs=True,
                        is_default=True  # These are default configurations
                    )
