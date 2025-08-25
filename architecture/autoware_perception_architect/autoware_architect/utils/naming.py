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

"""Naming utilities for the autoware architecture system."""

import re
from typing import Tuple

from ..exceptions import ValidationError


def pascal_to_snake(name: str) -> str:
    """Convert PascalCase to snake_case.
    
    Args:
        name: String in PascalCase format
        
    Returns:
        String in snake_case format
    """
    if not name:
        return ""
    
    # Insert underscore before uppercase letters that follow lowercase letters
    s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", name)
    # Insert underscore before uppercase letters that follow lowercase letters or digits
    return re.sub("([a-z0-9])([A-Z])", r"\1_\2", s1).lower()


def element_name_decode(element_name: str) -> Tuple[str, str]:
    """Decode element name into name and type.
    
    Args:
        element_name: Full element name like "ObjectDetector.module"
        
    Returns:
        Tuple of (element_name, element_type)
        
    Raises:
        ValidationError: If element name format is invalid
    """
    if not element_name or "." not in element_name:
        raise ValidationError(f"Invalid element name format: '{element_name}'. Expected format: 'Name.type'")

    parts = element_name.split(".")
    if len(parts) < 2:
        raise ValidationError(f"Invalid element name format: '{element_name}'. Expected format: 'Name.type'")

    name = parts[0]
    element_type = parts[1]

    # Valid element types
    valid_types = ["module", "pipeline", "parameter_set", "architecture"]
    if element_type not in valid_types:
        raise ValidationError(f"Invalid element type: '{element_type}'. Valid types: {valid_types}")

    return name, element_type


def snake_to_pascal(name: str) -> str:
    """Convert snake_case to PascalCase.
    
    Args:
        name: String in snake_case format
        
    Returns:
        String in PascalCase format
    """
    if not name:
        return ""
    
    components = name.split('_')
    return ''.join(word.capitalize() for word in components)
