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

from ..parsers.data_parser import ElementParser
from ..models.data_class import ElementData, ElementType
from ..exceptions import ValidationError

logger = logging.getLogger(__name__)

class ElementCollection:
    """Collection for managing multiple element data structures."""
    
    def __init__(self, config_yaml_file_paths: List[str]):
        # Replace list with dict as primary storage
        self.elements: Dict[str, ElementData] = {}  # full_name → ElementData
        self._type_map: Dict[str, Dict[str, ElementData]] = {
            ElementType.MODULE: {},
            ElementType.PIPELINE: {},
            ElementType.PARAMETER_SET: {},
            ElementType.ARCHITECTURE: {}
        }
        
        self.parser = ElementParser()
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
    
    def get_element_by_name(self, name: str) -> ElementData:
        """Get element by full name."""
        if name in self.elements:
            return self.elements[name]
        
        available_names = list(self.elements.keys())
        raise ValidationError(f"Element '{name}' not found. Available: {available_names}")
    
    def get_elements_by_type(self, element_type: str) -> List[ElementData]:
        """Get all elements of a specific type."""
        if element_type not in self._type_map:
            raise ValidationError(f"Invalid element type: {element_type}")
        return list(self._type_map[element_type].values())
    
    def get(self, name: str, default=None) -> Optional[ElementData]:
        """Get element by name with default value."""
        return self.elements.get(name, default)
    
    def get_architectures(self) -> Dict[str, ElementData]:
        """Get architectures as dictionary for easy lookup."""
        return self._type_map[ElementType.ARCHITECTURE].copy()
    
    def get_modules(self) -> Dict[str, ElementData]:
        """Get modules as dictionary for easy lookup."""
        return self._type_map[ElementType.MODULE].copy()
    
    def get_pipelines(self) -> Dict[str, ElementData]:
        """Get pipelines as dictionary for easy lookup."""
        return self._type_map[ElementType.PIPELINE].copy()
    
    def get_parameter_sets(self) -> Dict[str, ElementData]:
        """Get parameter sets as dictionary for easy lookup."""
        return self._type_map[ElementType.PARAMETER_SET].copy()