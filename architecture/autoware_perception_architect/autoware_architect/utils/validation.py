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

"""Validation utilities for configuration files."""

from typing import Dict, Any, List
import logging

from ..exceptions import ModuleConfigurationError, PipelineConfigurationError, ValidationError

logger = logging.getLogger(__name__)


def validate_module_config(config: Dict[str, Any]) -> bool:
    """Validate module configuration.
    
    Args:
        config: Module configuration dictionary
        
    Returns:
        True if configuration is valid
        
    Raises:
        ModuleConfigurationError: If configuration is invalid
    """
    required_fields = [
        "name",
        "launch", 
        "inputs",
        "outputs",
        "parameters",
        "configurations",
        "processes",
    ]
    
    # Check required fields
    for field in required_fields:
        if field not in config:
            raise ModuleConfigurationError(f"Required field '{field}' missing in module configuration")
    
    # Validate name format
    name = config.get("name", "")
    if not name.endswith(".module"):
        raise ModuleConfigurationError(f"Module name must end with '.module', got: '{name}'")
    
    # Validate launch configuration
    launch_config = config.get("launch", {})
    required_launch_fields = ["package"]
    for field in required_launch_fields:
        if field not in launch_config:
            raise ModuleConfigurationError(f"Required launch field '{field}' missing")
    
    # Validate container configuration
    if launch_config.get("use_container", False) and not launch_config.get("container_name"):
        raise ModuleConfigurationError("Container name is required when use_container is True")
    
    # Validate inputs/outputs are lists
    if not isinstance(config.get("inputs", []), list):
        raise ModuleConfigurationError("'inputs' must be a list")
    
    if not isinstance(config.get("outputs", []), list):
        raise ModuleConfigurationError("'outputs' must be a list")
    
    logger.debug(f"Module configuration validation passed for: {name}")
    return True


def validate_pipeline_config(config: Dict[str, Any]) -> bool:
    """Validate pipeline configuration.
    
    Args:
        config: Pipeline configuration dictionary
        
    Returns:
        True if configuration is valid
        
    Raises:
        PipelineConfigurationError: If configuration is invalid
    """
    required_fields = [
        "name",
        "depends",
        "nodes", 
        "external_interfaces",
        "connections",
        "parameters",
        "configurations",
    ]
    
    # Check required fields
    for field in required_fields:
        if field not in config:
            raise PipelineConfigurationError(f"Required field '{field}' missing in pipeline configuration")
    
    # Validate name format
    name = config.get("name", "")
    if not name.endswith(".pipeline"):
        raise PipelineConfigurationError(f"Pipeline name must end with '.pipeline', got: '{name}'")
    
    # Validate external interfaces
    external_interfaces = config.get("external_interfaces", {})
    required_interface_fields = ["input", "output"]
    optional_interface_fields = ["parameter"]
    
    # Check required fields
    for field in required_interface_fields:
        if field not in external_interfaces:
            raise PipelineConfigurationError(f"Required external interface field '{field}' missing")
    
    # Validate that if parameter field exists, it's a list
    if "parameter" in external_interfaces and not isinstance(external_interfaces["parameter"], list):
        raise PipelineConfigurationError("External interface 'parameter' field must be a list")
    
    # Validate connections are lists
    if not isinstance(config.get("connections", []), list):
        raise PipelineConfigurationError("'connections' must be a list")
    
    logger.debug(f"Pipeline configuration validation passed for: {name}")
    return True


def validate_parameter_set_config(config: Dict[str, Any]) -> bool:
    """Validate parameter set configuration.
    
    Args:
        config: Parameter set configuration dictionary
        
    Returns:
        True if configuration is valid
        
    Raises:
        ValidationError: If configuration is invalid
    """
    required_fields = ["name", "parameters"]
    
    # Check required fields
    for field in required_fields:
        if field not in config:
            raise ValidationError(f"Required field '{field}' missing in parameter set configuration")
    
    # Validate name format
    name = config.get("name", "")
    if not name.endswith(".parameter_set"):
        raise ValidationError(f"Parameter set name must end with '.parameter_set', got: '{name}'")
    
    # Validate parameters is a list
    if not isinstance(config.get("parameters", []), list):
        raise ValidationError("'parameters' must be a list")
    
    logger.debug(f"Parameter set configuration validation passed for: {name}")
    return True


def validate_architecture_config(config: Dict[str, Any]) -> bool:
    """Validate architecture configuration.
    
    Args:
        config: Architecture configuration dictionary
        
    Returns:
        True if configuration is valid
        
    Raises:
        ValidationError: If configuration is invalid
    """
    required_fields = ["name", "components", "connections"]
    
    # Check required fields
    for field in required_fields:
        if field not in config:
            raise ValidationError(f"Required field '{field}' missing in architecture configuration")
    
    # Validate name format
    name = config.get("name", "")
    if not name.endswith(".architecture"):
        raise ValidationError(f"Architecture name must end with '.architecture', got: '{name}'")
    
    # Validate components and connections are lists
    if not isinstance(config.get("components", []), list):
        raise ValidationError("'components' must be a list")
    
    if not isinstance(config.get("connections", []), list):
        raise ValidationError("'connections' must be a list")
    
    logger.debug(f"Architecture configuration validation passed for: {name}")
    return True
