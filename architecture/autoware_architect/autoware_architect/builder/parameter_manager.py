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
import re

from ..models.parameters import ParameterList, ParameterType, Parameter
from ..parsers.yaml_parser import yaml_parser

if TYPE_CHECKING:
    from .instances import Instance
    from .config_registry import ConfigRegistry

logger = logging.getLogger(__name__)


class ParameterManager:
    """Manages parameter operations for Instance objects.
    
    This class handles:
    1. Applying parameters from parameter sets to target instances
    2. Initializing node parameters from configuration
    3. Managing parameter values and parameters
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
        if self.instance.entity_type == "node" and self.instance.configuration:
            launch_config = self.instance.configuration.launch
            package_name = launch_config.get("package")
        
        result = []
        for param in self.parameter_files.get_parameter_files_ordered():
            # We use the original value (unresolved path) here because the launcher generator
            # might handle resolution or we want to preserve $(find-pkg-share) syntax for launch files.
            # However, if we want to support absolute paths we resolved earlier, we should use them?
            # The current design seems to re-resolve at launch generation time. 
            # Let's keep using _resolve_parameter_file_path but we need to be careful about
            # what 'config_registry' context we have here. 
            # Since we don't pass config_registry here, we can only resolve based on what we have.
            # But wait, we need package paths to resolve $(find-pkg-share).
            # If we don't have config_registry here, we can't resolve correctly if it relies on it.
            # For now, we'll assume the value is what we want in the launch file 
            # (which might include $(find-pkg-share)).
            
            # If the value is an absolute path (resolved during init), use it.
            # If it is a relative path and we have package info, we might produce $(find-pkg-share ...).
             
             resolved_path = self._resolve_parameter_file_path(param.value, package_name, param.is_default)
             result.append({"path": resolved_path})
        
        return result
    
    def get_parameters_for_launch(self) -> List[Dict[str, Any]]:
        """Get parameters formatted for launcher generation.
        
        Returns list of dicts with 'name' and 'value' keys, filtering out 'none' values.
        """
        result = []
        for param in self.parameter_files.get_parameters():
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
                                     is_default: bool = False, config_registry: Optional['ConfigRegistry'] = None) -> str:
        """Resolve parameter file path with package prefix if needed.
        
        Args:
            path: The parameter file path
            package_name: The ROS package name for default parameters
            is_default: Whether this is a default parameter file
            config_registry: Registry to look up package paths
            
        Returns:
            Resolved path with package prefix if applicable
        """

        if path is None:
            raise ValueError(f"path is None. package_name: {package_name}, node_namespace: {self.instance.namespace_str}, path: {path}")
        
        # If path already starts with $( or /, don't add prefix
        if path.startswith('$(') or path.startswith('/'):
            return path
        
        # If we have config_registry and it's a default param, try to resolve to absolute path
        if is_default and package_name and config_registry:
            pkg_path = config_registry.get_package_path(package_name)
            if pkg_path:
                return os.path.join(pkg_path, path)

        # For default parameters without registry (or fallback), add package prefix for launch file
        if is_default and package_name:
            return f"$(find-pkg-share {package_name})/{path}"
        
        # For overrides or when no package name, return as-is
        return path

    # =========================================================================
    # Parameter Application (from parameter sets)
    # =========================================================================
    
    def apply_node_parameters(self, node_namespace: str, parameter_files: list, parameters: list, config_registry: Optional['ConfigRegistry'] = None):
        """Apply parameters directly to a target node using new parameter set format.
        
        This method finds a node by its absolute namespace and applies both parameter_files 
        and parameters directly to it. Parameters will override parameter_files.
        
        Args:
            node_namespace: Absolute namespace path to the target node 
                           (e.g., "/perception/object_recognition/node_tracker")
            parameter_files: List of parameter file mappings 
                            (e.g., [{"model_param_path": "path/to/file.yaml"}])
            parameters: List of direct parameters
                           (e.g., [{"name": "build_only", "type": "bool", "value": false}])
            config_registry: Registry for resolving paths
        """
        target_instance = self._find_node_by_namespace(node_namespace)
        if target_instance is None:
            logger.warning(f"Target node not found: {node_namespace}")
            return
            
        if target_instance.entity_type != "node":
            logger.warning(f"Target node is not a node: {node_namespace} (type: {target_instance.entity_type})")
            return
        
        logger.info(f"Applying parameters to node: {node_namespace}")
            
        # Apply parameter files first (as overrides, not defaults)
        if parameter_files:
            for param_file_mapping in parameter_files:
                for param_name, param_path in param_file_mapping.items():
                    target_instance.parameter_manager.parameter_files.set_parameter(
                        param_name,
                        param_path,
                        param_type=ParameterType.PARAMETER_FILE,
                        data_type="path",
                        allow_substs=True,
                        is_default=False  # Parameter set overrides are not defaults
                    )
                    # Load the parameters from this file as well
                    target_instance.parameter_manager._load_parameters_from_file(
                        param_path, 
                        is_default=False, 
                        config_registry=config_registry
                    )
        
        # Apply parameters (these override parameter files)
        if parameters:
            for param in parameters:
                param_name = param.get("name")
                param_type = param.get("type", "string")
                param_value = param.get("value")

                target_instance.parameter_manager.parameter_files.set_parameter(
                    param_name,
                    param_value,
                    param_type=ParameterType.PARAMETER,
                    data_type=param_type,
                    allow_substs=True,
                    is_default=False  # Parameter set are not defaults
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
            
        # If we're at the deployment root (entity_type == "system"),
        # search in its children directly since parameter_set paths don't include the deployment name
        if current_instance.entity_type == "system":
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
    # Node Parameter Initialization
    # =========================================================================
    
    def initialize_node_parameters(self, config_registry: Optional['ConfigRegistry'] = None):
        """Initialize parameters for node entity during node configuration.
        This method initializes both default parameter_files and default parameters
        from the node's configuration file.
        """
        if self.instance.entity_type != "node":
            return
            
        package_name = None
        if self.instance.configuration and hasattr(self.instance.configuration, 'launch'):
            package_name = self.instance.configuration.launch.get("package")
        
        # 1. Set default parameter_files from node configuration
        if hasattr(self.instance.configuration, 'parameter_files') and self.instance.configuration.parameter_files:
            for cfg_param in self.instance.configuration.parameter_files:
                param_name = cfg_param.get("name")
                param_value = cfg_param.get("value", cfg_param.get("default"))
                param_schema = cfg_param.get("schema")

                if param_name is None or param_value is None:
                    raise ValueError(f"param_name or param_value is None. namespace: {self.instance.namespace_str}, parameter_files: {self.instance.configuration.parameter_files}")
                
                # Load individual parameters from this file
                self._load_parameters_from_file(
                    param_value, 
                    package_name=package_name, 
                    is_default=True, 
                    config_registry=config_registry
                )
        
        # 2. Set default parameters from node parameters
        if hasattr(self.instance.configuration, 'parameters') and self.instance.configuration.parameters:
            for cfg_param in self.instance.configuration.parameters:
                param_name = cfg_param.get("name")
                param_value = cfg_param.get("value", cfg_param.get("default"))
                param_type = cfg_param.get("type", "string")

                if param_name is None or param_value is None:
                    raise ValueError(f"param_name or param_value is None. namespace: {self.instance.namespace_str}, parameter_files: {self.instance.configuration.parameter_files}")

                # Only set if a default value is provided
                if param_value is not None:
                    self.parameter_files.set_parameter(
                        param_name,
                        param_value,
                        param_type=ParameterType.PARAMETER,
                        data_type=param_type,
                        allow_substs=True,
                        is_default=True  # These are default parameters
                    )

    def _flatten_parameters(self, params: Dict[str, Any], parent_key: str = "", separator: str = ".") -> Dict[str, Any]:
        """Flatten nested dictionary into dot-separated keys.
        
        Args:
            params: The dictionary to flatten
            parent_key: Key prefix for recursion
            separator: Separator for keys
            
        Returns:
            Dict with flattened keys
        """
        items = {}
        for k, v in params.items():
            new_key = f"{parent_key}{separator}{k}" if parent_key else k
            
            if isinstance(v, dict):
                items.update(self._flatten_parameters(v, new_key, separator))
            else:
                items[new_key] = v
        return items

    def _load_parameters_from_file(self, file_path: str, package_name: Optional[str] = None, 
                                  is_default: bool = False, config_registry: Optional['ConfigRegistry'] = None):
        """Load parameters from a YAML file and add them to the parameter list."""
        if not config_registry:
            logger.debug(f"Skipping parameter file load for {file_path}: No config_registry provided")
            return

        try:
            # Resolve the full path
            resolved_path = self._resolve_parameter_file_path(file_path, package_name, is_default, config_registry)
            
            # Skip if we couldn't resolve to an absolute path or it involves substitutions
            if not resolved_path or resolved_path.startswith("$") or not os.path.isabs(resolved_path):
                logger.debug(f"Skipping parameter file load for {file_path}: Could not resolve to absolute path ({resolved_path})")
                return
            
            if not os.path.exists(resolved_path):
                logger.warning(f"Parameter file not found: {resolved_path}")
                return
                
            logger.debug(f"Loading parameters from file: {resolved_path}")
            data = yaml_parser.load_config(resolved_path)
            
            if not data:
                return

            # Parse ROS 2 parameter file structure
            # Format:
            # node_name:
            #   ros__parameters:
            #     param_name: value
            
            node_name = self.instance.name
            
            # Find relevant sections
            for key, value in data.items():
                # Check if key matches node name or wildcard
                # Simple matching: exact match or /**
                # Could look into more complex ROS 2 matching rules if needed
                if key == "/**" or key == f"/{node_name}" or key == node_name:
                    if "ros__parameters" in value:
                        # Flatten the nested structure
                        flattened_params = self._flatten_parameters(value["ros__parameters"])
                        
                        for p_name, p_value in flattened_params.items():
                            # Infer type
                            p_type = "string"
                            if isinstance(p_value, bool):
                                p_type = "bool"
                            elif isinstance(p_value, int):
                                p_type = "int"
                            elif isinstance(p_value, float):
                                p_type = "double"
                            elif isinstance(p_value, list):
                                # arrays
                                if len(p_value) > 0:
                                    if isinstance(p_value[0], int): p_type = "int_array"
                                    elif isinstance(p_value[0], float): p_type = "double_array"
                                    elif isinstance(p_value[0], str): p_type = "string_array"
                                    elif isinstance(p_value[0], bool): p_type = "bool_array"
                            
                            self.parameter_files.set_parameter(
                                p_name,
                                p_value,
                                param_type=ParameterType.PARAMETER,
                                data_type=p_type,
                                is_default=is_default
                            )
        except Exception as e:
            logger.warning(f"Failed to load parameters from file {file_path}: {e}")
