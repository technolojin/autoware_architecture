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

from typing import List, Dict, Any, Tuple
from pathlib import Path
import logging

from ..parsers.yaml_parser import yaml_parser
from ..exceptions import ValidationError

logger = logging.getLogger(__name__)

debug_mode = True

class ElementType:
    """Constants for element types."""
    MODULE = "module"
    PIPELINE = "pipeline"
    PARAMETER_SET = "parameter_set"
    ARCHITECTURE = "architecture"

    @classmethod
    def get_all_types(cls) -> List[str]:
        """Get all valid element types."""
        return [cls.MODULE, cls.PIPELINE, cls.PARAMETER_SET, cls.ARCHITECTURE]

def element_name_decode(element_name: str) -> Tuple[str, str]:
    """Decode element name into name and type components.
    
    Args:
        element_name: Full element name (e.g., "ObjectDetector.module")
        
    Returns:
        Tuple of (element_name, element_type)
        
    Raises:
        ValidationError: If element name format is invalid
        
    Example:
        >>> name, type = element_name_decode("ObjectDetector.module")
        >>> print(f"Name: {name}, Type: {type}")
        Name: ObjectDetector, Type: module
    """
    if not element_name or not isinstance(element_name, str):
        raise ValidationError(f"Element name must be a non-empty string, got: {element_name}")
    
    if "." not in element_name:
        raise ValidationError(f"Invalid element name format: '{element_name}'. Expected format: 'name.type'")

    parts = element_name.split(".")
    if len(parts) != 2:
        raise ValidationError(f"Invalid element name format: '{element_name}'. Expected exactly one dot separator")

    name, element_type = parts
    
    if not name.strip():
        raise ValidationError(f"Element name cannot be empty in: '{element_name}'")
    
    if not element_type.strip():
        raise ValidationError(f"Element type cannot be empty in: '{element_name}'")

    # Check the element type
    if element_type not in ElementType.get_all_types():
        raise ValidationError(f"Invalid element type: '{element_type}'. Valid types: {ElementType.get_all_types()}")

    return name.strip(), element_type.strip()


# classes for architecture configuration

class ConfigValidator:
    """Unified configuration validator for all element types."""
    
    @staticmethod
    def validate_element_type(element_type: str, expected_type: str, file_path: str) -> None:
        """Validate that the element type matches expected type."""
        if element_type != expected_type:
            raise ValidationError(
                f"Invalid element type '{element_type}'. Expected '{expected_type}'. "
                f"File: {file_path}"
            )
    
    @staticmethod
    def validate_required_fields(config: Dict[str, Any], required_fields: List[str], expected_type: str, file_path: str) -> None:
        """Validate that all required fields are present."""
        missing_fields = []
        for field in required_fields:
            if field not in config:
                missing_fields.append(field)
        
        if missing_fields:
            raise ValidationError(
                f"Missing required fields {missing_fields} in {expected_type} "
                f"configuration. File: {file_path}"
            )
    
    @staticmethod
    def validate_schema(config: Dict[str, Any], schema: Dict[str, Any], file_path: str) -> None:
        """Validate configuration against schema."""
        errors = []
        
        # Check field types
        field_types = schema.get('properties', {})
        for field, field_schema in field_types.items():
            if field in config:
                expected_type = field_schema.get('type')
                if expected_type and not ConfigValidator._validate_type(config[field], expected_type):
                    errors.append(f"Field '{field}' has invalid type. Expected: {expected_type}")
        
        if errors:
            error_msg = f"Schema validation failed for {file_path}:\n" + "\n".join(f"  - {error}" for error in errors)
            raise ValidationError(error_msg)
    
    @staticmethod
    def _validate_type(value: Any, expected_type: str) -> bool:
        """Validate that a value matches the expected type."""
        type_map = {
            'string': str,
            'integer': int,
            'number': (int, float),
            'boolean': bool,
            'array': list,
            'object': dict,
            'object_or_array': (dict, list),  # Allow both objects and arrays
            'nullable_object': (dict, type(None)),  # Allow object or null
            'nullable_array': (list, type(None)),   # Allow array or null
        }
        
        if expected_type in type_map:
            expected_types = type_map[expected_type]
            if isinstance(expected_types, tuple):
                return isinstance(value, expected_types)
            else:
                return isinstance(value, expected_types)
        return True
    
    @staticmethod
    def validate_all(config: Dict[str, Any], element_type: str, expected_type: str, 
                     required_fields: List[str], schema: Dict[str, Any], file_path: str) -> None:
        """Perform complete validation - element type, required fields, and schema validation."""
        ConfigValidator.validate_element_type(element_type, expected_type, file_path)
        ConfigValidator.validate_required_fields(config, required_fields, expected_type, file_path)
        ConfigValidator.validate_schema(config, schema, file_path)

class Element:
    """Base class for all configuration elements."""
    
    # Override in subclasses to define required fields and schema
    REQUIRED_FIELDS = ["name"]
    SCHEMA_PROPERTIES = {
        'name': {'type': 'string'}
    }
    
    def __init__(self, config_yaml_dir: str, expected_element_type: str):
        self.config_yaml_dir = Path(config_yaml_dir)
        self.expected_element_type = expected_element_type
        
        # Load and validate configuration
        self.config_yaml = self._load_config()
        self._validate_basic_structure()
        
        # Parse element name and type
        self.full_name = self.config_yaml.get("name")
        self.name, self.type = element_name_decode(self.full_name)
        
        # Validate element type
        if self.type not in ElementType.get_all_types():
            raise ValidationError(f"Invalid element type: '{self.type}'. File: {self.config_yaml_dir}")
        
        # Run complete validation using ConfigValidator
        ConfigValidator.validate_all(
            config=self.config_yaml,
            element_type=self.type,
            expected_type=self.expected_element_type,
            required_fields=self.REQUIRED_FIELDS,
            schema=self.get_schema(),
            file_path=str(self.config_yaml_dir)
        )
        
        self.check_config()

    def get_schema(self) -> Dict[str, Any]:
        """Get schema definition for this element type."""
        return {
            'properties': self.SCHEMA_PROPERTIES
        }

    def _load_config(self) -> Dict[str, Any]:
        """Load YAML configuration using the proper parser."""
        if not self.config_yaml_dir.is_file():
            raise ValidationError(f"Config YAML file does not exist: {self.config_yaml_dir}")

        try:
            return yaml_parser.load_config(str(self.config_yaml_dir))
        except Exception as e:
            logger.error(f"Failed to load config from {self.config_yaml_dir}: {e}")
            raise ValidationError(f"Error parsing YAML file {self.config_yaml_dir}: {e}")
    
    def _validate_basic_structure(self) -> None:
        """Validate basic structure requirements."""
        if not self.config_yaml:
            raise ValidationError(f"Empty configuration file: {self.config_yaml_dir}")
        
        if "name" not in self.config_yaml:
            raise ValidationError(f"Field 'name' is required in element configuration. File: {self.config_yaml_dir}")
    
    def check_config(self) -> bool:
        """Override in subclasses for additional validation."""
        return True

class ModuleElement(Element):
    """Module configuration element."""
    
    REQUIRED_FIELDS = [
        "name",
        "launch",
        "inputs",
        "outputs",
        "parameters",
        "configurations",
        "processes",
    ]
    
    SCHEMA_PROPERTIES = {
        'name': {'type': 'string'},
        'launch': {'type': 'object'},
        'inputs': {'type': 'array'},
        'outputs': {'type': 'array'},
        'parameters': {'type': 'object_or_array'},  # Allow both {} and []
        'configurations': {'type': 'object_or_array'},  # Allow both {} and []
        'processes': {'type': 'array'}
    }
    
    def __init__(self, config_yaml_dir: str):
        super().__init__(config_yaml_dir, ElementType.MODULE)

class PipelineElement(Element):
    """Pipeline configuration element."""
    
    REQUIRED_FIELDS = [
        "name",
        "depends",
        "nodes",
        "external_interfaces",
        "connections",
        "parameters",
        "configurations",
    ]
    
    SCHEMA_PROPERTIES = {
        'name': {'type': 'string'},
        'depends': {'type': 'array'},
        'nodes': {'type': 'array'},
        'external_interfaces': {'type': 'object_or_array'},
        'connections': {'type': 'array'},
        'parameters': {'type': 'object_or_array'},
        'configurations': {'type': 'object_or_array'},
    }
    
    def __init__(self, config_yaml_dir: str):
        super().__init__(config_yaml_dir, ElementType.PIPELINE)

class ParameterSetElement(Element):
    """Parameter set configuration element."""
    
    REQUIRED_FIELDS = [
        "name",
        "parameters",
    ]
    
    SCHEMA_PROPERTIES = {
        'name': {'type': 'string'},
        'parameters': {'type': 'object_or_array'},
    }
    
    def __init__(self, config_yaml_dir: str):
        super().__init__(config_yaml_dir, ElementType.PARAMETER_SET)

class ArchitectureElement(Element):
    """Architecture configuration element."""
    
    REQUIRED_FIELDS = [
        "name",
        "components",
        "connections",
    ]
    
    SCHEMA_PROPERTIES = {
        'name': {'type': 'string'},
        'components': {'type': 'array'},
        'connections': {'type': 'array'},
    }
    
    def __init__(self, config_yaml_dir: str):
        super().__init__(config_yaml_dir, ElementType.ARCHITECTURE)


class ElementFactory:
    """Factory for creating element instances based on type."""
    
    _element_classes = {
        ElementType.MODULE: ModuleElement,
        ElementType.PIPELINE: PipelineElement,
        ElementType.PARAMETER_SET: ParameterSetElement,
        ElementType.ARCHITECTURE: ArchitectureElement,
    }
    
    @classmethod
    def create_element(cls, config_yaml_dir: str) -> Element:
        """Create an element instance based on the configuration type.
        
        Args:
            config_yaml_dir: Path to the configuration file
            
        Returns:
            Appropriate element instance
            
        Raises:
            ValidationError: If element type is invalid or file cannot be parsed
        """
        # First load the config to determine the type
        config_path = Path(config_yaml_dir)
        if not config_path.is_file():
            raise ValidationError(f"Config YAML file does not exist: {config_yaml_dir}")

        try:
            config = yaml_parser.load_config(str(config_path))
        except Exception as e:
            raise ValidationError(f"Error parsing YAML file {config_yaml_dir}: {e}")
        
        if not config or "name" not in config:
            raise ValidationError(f"Invalid configuration: missing 'name' field in {config_yaml_dir}")
        
        # Parse element name to get type
        full_name = config.get("name")
        name, element_type = element_name_decode(full_name)
        
        # Create the specific element type
        if element_type in cls._element_classes:
            return cls._element_classes[element_type](config_yaml_dir)
        else:
            raise ValidationError(f"Unknown element type: {element_type}")


class ElementList:
    """Container for managing multiple elements."""
    
    def __init__(self, config_yaml_file_dirs: List[str]):
        self.elements: List[Element] = []
        self._element_map: Dict[str, Element] = {}
        
        try:
            self._load_elements(config_yaml_file_dirs)
        except Exception as e:
            logger.error(f"Failed to load elements: {e}")
            raise

    def _load_elements(self, config_yaml_file_dirs: List[str]):
        """Load elements from configuration files."""
        for config_yaml_file_dir in config_yaml_file_dirs:
            if debug_mode:
                logger.info(f"Loading element from: {config_yaml_file_dir}")
            
            try:
                element = ElementFactory.create_element(config_yaml_file_dir)
                
                # Check for duplicates
                if element.full_name in self._element_map:
                    existing_element = self._element_map[element.full_name]
                    raise ValidationError(
                        f"Duplicate element '{element.full_name}' found:\n"
                        f"  New: {element.config_yaml_dir}\n"
                        f"  Existing: {existing_element.config_yaml_dir}"
                    )
                
                # Add to collections
                self.elements.append(element)
                self._element_map[element.full_name] = element
                
            except Exception as e:
                logger.error(f"Failed to load element from {config_yaml_file_dir}: {e}")
                raise
    
    def get_element_by_name(self, name: str) -> Element:
        """Get element by full name."""
        if name in self._element_map:
            return self._element_map[name]
        
        available_names = list(self._element_map.keys())
        raise ValidationError(f"Element '{name}' not found. Available: {available_names}")
    
    def get_elements_by_type(self, element_type: str) -> List[Element]:
        """Get all elements of a specific type."""
        return [element for element in self.elements if element.type == element_type]

    def get_module_list(self) -> List[str]:
        """Get list of module configuration file paths."""
        return [str(element.config_yaml_dir) for element in self.elements if element.type == ElementType.MODULE]

    def get_pipeline_list(self) -> List[str]:
        """Get list of pipeline configuration file paths."""
        return [str(element.config_yaml_dir) for element in self.elements if element.type == ElementType.PIPELINE]

    def get_parameter_set_list(self) -> List[str]:
        """Get list of parameter set configuration file paths."""
        return [str(element.config_yaml_dir) for element in self.elements if element.type == ElementType.PARAMETER_SET]

    def get_architecture_list(self) -> List[str]:
        """Get list of architecture configuration file paths."""
        return [str(element.config_yaml_dir) for element in self.elements if element.type == ElementType.ARCHITECTURE]


class BaseElementContainer:
    """Base class for element containers to reduce code duplication."""
    
    def __init__(self, element_paths: List[str], element_class, container_name: str):
        self.elements: List[Element] = []
        self._element_map: Dict[str, Element] = {}
        self.container_name = container_name
        
        for element_path in element_paths:
            element = element_class(element_path)
            self.elements.append(element)
            self._element_map[element.name] = element

    def get(self, element_name: str) -> Element:
        """Get element by name."""
        if element_name in self._element_map:
            return self._element_map[element_name]
        
        available_elements = list(self._element_map.keys())
        raise ValidationError(
            f"{self.container_name} '{element_name}' not found. Available: {available_elements}"
        )


class ModuleList(BaseElementContainer):
    """Container for managing module elements."""
    
    def __init__(self, module_list: List[str]):
        super().__init__(module_list, ModuleElement, "Module")


class PipelineList(BaseElementContainer):
    """Container for managing pipeline elements."""
    
    def __init__(self, pipeline_list: List[str]):
        super().__init__(pipeline_list, PipelineElement, "Pipeline")


class ParameterSetList(BaseElementContainer):
    """Container for managing parameter set elements."""
    
    def __init__(self, parameter_set_list: List[str]):
        super().__init__(parameter_set_list, ParameterSetElement, "Parameter set")


class ArchitectureList(BaseElementContainer):
    """Container for managing architecture elements."""
    
    def __init__(self, architecture_list: List[str]):
        super().__init__(architecture_list, ArchitectureElement, "Architecture")

    def get(self, architecture_name: str) -> ArchitectureElement:
        """Get architecture by name with special handling for full names."""
        # Handle full name format
        if "." in architecture_name:
            architecture_name, _ = element_name_decode(architecture_name)
        
        return super().get(architecture_name)