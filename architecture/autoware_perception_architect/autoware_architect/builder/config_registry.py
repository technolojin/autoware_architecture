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

from typing import List, Dict, Optional
import logging

from ..parsers.data_parser import ConfigParser
from ..models.config import Config, ConfigType, ModuleConfig, PipelineConfig, ParameterSetConfig, ArchitectureConfig
from ..exceptions import ValidationError

logger = logging.getLogger(__name__)

class ConfigRegistry:
    """Collection for managing multiple element data structures with efficient lookup methods."""
    
    def __init__(self, config_yaml_file_paths: List[str]):
        # Replace list with dict as primary storage
        self.elements: Dict[str, Config] = {}  # full_name → Config
        self._type_map: Dict[str, Dict[str, Config]] = {
            ConfigType.MODULE: {},
            ConfigType.PIPELINE: {},
            ConfigType.PARAMETER_SET: {},
            ConfigType.ARCHITECTURE: {}
        }
        
        self.parser = ConfigParser()
        self._load_elements(config_yaml_file_paths)
    
    def _load_elements(self, config_yaml_file_paths: List[str]) -> None:
        """Load elements from configuration files."""
        for file_path in config_yaml_file_paths:
            logger.debug(f"Loading element from: {file_path}")
            
            try:
                element_data = self.parser.parse_element_file(file_path)
                
                # Check for duplicates
                if element_data.full_name in self.elements:
                    existing = self.elements[element_data.name]
                    raise ValidationError(
                        f"Duplicate element '{element_data.full_name}' found:\n"
                        f"  New: {element_data.file_path}\n"
                        f"  Existing: {existing.file_path}"
                    )
                
                # Add to collections
                self.elements[element_data.full_name] = element_data
                self._type_map[element_data.element_type][element_data.name] = element_data
                
            except Exception as e:
                logger.error(f"Failed to load element from {file_path}: {e}")
                raise
    
    def get(self, name: str, default=None) -> Optional[Config]:
        """Get element by name with default value."""
        return self.elements.get(name, default)
    
    # Enhanced methods for type-safe element access
    def get_module(self, name: str) -> ModuleConfig:
        """Get a module element by name."""
        element = self._type_map[ConfigType.MODULE].get(name)
        if element is None:
            available = list(self._type_map[ConfigType.MODULE].keys())
            raise ValidationError(f"Module '{name}' not found. Available modules: {available}")
        return element
    
    def get_pipeline(self, name: str) -> PipelineConfig:
        """Get a pipeline element by name."""
        element = self._type_map[ConfigType.PIPELINE].get(name)
        if element is None:
            available = list(self._type_map[ConfigType.PIPELINE].keys())
            raise ValidationError(f"Pipeline '{name}' not found. Available pipelines: {available}")
        return element
    
    def get_parameter_set(self, name: str) -> ParameterSetConfig:
        """Get a parameter set element by name."""
        element = self._type_map[ConfigType.PARAMETER_SET].get(name)
        if element is None:
            available = list(self._type_map[ConfigType.PARAMETER_SET].keys())
            raise ValidationError(f"Parameter set '{name}' not found. Available parameter sets: {available}")
        return element
    
    def get_architecture(self, name: str) -> ArchitectureConfig:
        """Get an architecture element by name."""
        element = self._type_map[ConfigType.ARCHITECTURE].get(name)
        if element is None:
            available = list(self._type_map[ConfigType.ARCHITECTURE].keys())
            raise ValidationError(f"Architecture '{name}' not found. Available architectures: {available}")
        return element
    
    def get_element_by_type(self, name: str, element_type: str) -> Config:
        """Get an element by name and type."""
        if element_type == ConfigType.MODULE:
            return self.get_module(name)
        elif element_type == ConfigType.PIPELINE:
            return self.get_pipeline(name)
        elif element_type == ConfigType.PARAMETER_SET:
            return self.get_parameter_set(name)
        elif element_type == ConfigType.ARCHITECTURE:
            return self.get_architecture(name)
        else:
            raise ValidationError(f"Unknown element type: {element_type}")

