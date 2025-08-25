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

"""Element models for architecture configuration."""

import logging
from pathlib import Path
from typing import List, Dict, Any, Union
from abc import ABC, abstractmethod

from ..parsers.yaml_parser import yaml_parser
from ..utils.naming import element_name_decode
from ..utils.validation import (
    validate_module_config, validate_pipeline_config, 
    validate_parameter_set_config, validate_architecture_config
)
from ..exceptions import ValidationError, ModuleConfigurationError, PipelineConfigurationError

logger = logging.getLogger(__name__)


class Element(ABC):
    """Base class for all architecture elements."""
    
    def __init__(self, config_path: Union[str, Path]):
        """Initialize element from configuration file.
        
        Args:
            config_path: Path to configuration YAML file
        """
        self.config_path = Path(config_path)
        self.config = yaml_parser.load_config(self.config_path)
        
        # Validate name field
        if "name" not in self.config:
            raise ValidationError(f"Field 'name' is required in element configuration: {self.config_path}")
        
        self.full_name = self.config["name"]
        self.name, self.type = element_name_decode(self.full_name)
        
        # Validate configuration
        self.validate_config()
        
        logger.debug(f"Created {self.type} element: {self.name}")
    
    @abstractmethod
    def validate_config(self) -> bool:
        """Validate element configuration.
        
        Returns:
            True if configuration is valid
            
        Raises:
            ValidationError: If configuration is invalid
        """
        pass


class ModuleElement(Element):
    """Module element representing a single ROS node."""
    
    def validate_config(self) -> bool:
        """Validate module configuration."""
        if self.type != "module":
            raise ModuleConfigurationError(f"Expected module type, got: {self.type}")
        
        return validate_module_config(self.config)
    
    @property
    def launch_config(self) -> Dict[str, Any]:
        """Get launch configuration."""
        return self.config.get("launch", {})
    
    @property
    def package_name(self) -> str:
        """Get ROS package name."""
        return self.launch_config.get("package", "")
    
    @property
    def plugin_name(self) -> str:
        """Get plugin name for composable nodes."""
        return self.launch_config.get("plugin", "")
    
    @property
    def inputs(self) -> List[Dict[str, Any]]:
        """Get input interface definitions."""
        return self.config.get("inputs", [])
    
    @property
    def outputs(self) -> List[Dict[str, Any]]:
        """Get output interface definitions."""
        return self.config.get("outputs", [])
    
    @property
    def parameters(self) -> List[Dict[str, Any]]:
        """Get parameter definitions."""
        return self.config.get("parameters", [])
    
    @property
    def configurations(self) -> List[Dict[str, Any]]:
        """Get configuration definitions."""
        return self.config.get("configurations", [])
    
    @property
    def processes(self) -> List[Dict[str, Any]]:
        """Get process definitions."""
        return self.config.get("processes", [])
    
    @property
    def use_container(self) -> bool:
        """Check if module should use composable node container."""
        return self.launch_config.get("use_container", False)
    
    @property
    def container_name(self) -> str:
        """Get container name for composable nodes."""
        return self.launch_config.get("container_name", "")
    
    @property
    def node_output(self) -> str:
        """Get node output mode."""
        return self.launch_config.get("node_output", "screen")


class PipelineElement(Element):
    """Pipeline element representing a collection of connected modules."""
    
    def validate_config(self) -> bool:
        """Validate pipeline configuration."""
        if self.type != "pipeline":
            raise PipelineConfigurationError(f"Expected pipeline type, got: {self.type}")
        
        return validate_pipeline_config(self.config)
    
    @property
    def depends(self) -> List[str]:
        """Get dependency list."""
        return self.config.get("depends", [])
    
    @property
    def nodes(self) -> List[Dict[str, Any]]:
        """Get node definitions."""
        return self.config.get("nodes", [])
    
    @property
    def external_interfaces(self) -> Dict[str, Any]:
        """Get external interface definitions."""
        return self.config.get("external_interfaces", {})
    
    @property
    def connections(self) -> List[Dict[str, Any]]:
        """Get connection definitions."""
        return self.config.get("connections", [])
    
    @property
    def parameters(self) -> List[Dict[str, Any]]:
        """Get parameter definitions."""
        return self.config.get("parameters", [])
    
    @property
    def configurations(self) -> List[Dict[str, Any]]:
        """Get configuration definitions."""
        return self.config.get("configurations", [])


class ParameterSetElement(Element):
    """Parameter set element representing a collection of parameters."""
    
    def validate_config(self) -> bool:
        """Validate parameter set configuration."""
        if self.type != "parameter_set":
            raise ValidationError(f"Expected parameter_set type, got: {self.type}")
        
        return validate_parameter_set_config(self.config)
    
    @property
    def parameters(self) -> List[Dict[str, Any]]:
        """Get parameter definitions."""
        return self.config.get("parameters", [])


class ArchitectureElement(Element):
    """Architecture element representing the top-level system architecture."""
    
    def validate_config(self) -> bool:
        """Validate architecture configuration."""
        if self.type != "architecture":
            raise ValidationError(f"Expected architecture type, got: {self.type}")
        
        return validate_architecture_config(self.config)
    
    @property
    def components(self) -> List[Dict[str, Any]]:
        """Get component definitions."""
        return self.config.get("components", [])
    
    @property
    def connections(self) -> List[Dict[str, Any]]:
        """Get connection definitions."""
        return self.config.get("connections", [])


class ElementFactory:
    """Factory for creating elements based on their type."""
    
    _element_classes = {
        "module": ModuleElement,
        "pipeline": PipelineElement,
        "parameter_set": ParameterSetElement,
        "architecture": ArchitectureElement,
    }
    
    @classmethod
    def create_element(cls, config_path: Union[str, Path]) -> Element:
        """Create element from configuration file.
        
        Args:
            config_path: Path to configuration file
            
        Returns:
            Element instance
            
        Raises:
            ValidationError: If element type is unknown
        """
        # Load config to determine type
        config = yaml_parser.load_config(config_path)
        if "name" not in config:
            raise ValidationError(f"Field 'name' is required in element configuration: {config_path}")
        
        _, element_type = element_name_decode(config["name"])
        
        if element_type not in cls._element_classes:
            raise ValidationError(f"Unknown element type: {element_type}")
        
        element_class = cls._element_classes[element_type]
        return element_class(config_path)


class ElementList:
    """Container for managing multiple elements."""
    
    def __init__(self, config_paths: List[Union[str, Path]]):
        """Initialize element list from configuration paths.
        
        Args:
            config_paths: List of paths to configuration files
        """
        self.elements: List[Element] = []
        self._load_elements(config_paths)
    
    def _load_elements(self, config_paths: List[Union[str, Path]]):
        """Load elements from configuration paths."""
        for config_path in config_paths:
            try:
                element = ElementFactory.create_element(config_path)
                self.elements.append(element)
                logger.debug(f"Loaded element: {element.full_name}")
            except Exception as exc:
                logger.error(f"Failed to load element from {config_path}: {exc}")
                raise
    
    def get_modules(self) -> List[ModuleElement]:
        """Get all module elements."""
        return [e for e in self.elements if isinstance(e, ModuleElement)]
    
    def get_pipelines(self) -> List[PipelineElement]:
        """Get all pipeline elements."""
        return [e for e in self.elements if isinstance(e, PipelineElement)]
    
    def get_parameter_sets(self) -> List[ParameterSetElement]:
        """Get all parameter set elements."""
        return [e for e in self.elements if isinstance(e, ParameterSetElement)]
    
    def get_architectures(self) -> List[ArchitectureElement]:
        """Get all architecture elements."""
        return [e for e in self.elements if isinstance(e, ArchitectureElement)]
    
    def find_by_name(self, name: str) -> Element:
        """Find element by name.
        
        Args:
            name: Element name (with or without type suffix)
            
        Returns:
            Element instance
            
        Raises:
            ValidationError: If element not found
        """
        # If name includes type suffix, search by full name
        if "." in name:
            for element in self.elements:
                if element.full_name == name:
                    return element
        else:
            # Search by base name
            for element in self.elements:
                if element.name == name:
                    return element
        
        raise ValidationError(f"Element not found: {name}")
    
    def find_by_type(self, element_type: str) -> List[Element]:
        """Find elements by type.
        
        Args:
            element_type: Type of element to find
            
        Returns:
            List of matching elements
        """
        return [e for e in self.elements if e.type == element_type]
