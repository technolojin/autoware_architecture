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

"""Parameter models for configuration management."""

from typing import Dict, Any, List, Optional
import logging

from ..exceptions import ValidationError

logger = logging.getLogger(__name__)


class Parameter:
    """Represents a single parameter with name and value."""
    
    def __init__(self, name: str, value: Any):
        """Initialize parameter.
        
        Args:
            name: Parameter name
            value: Parameter value
        """
        self.name = name
        self.value = value
    
    def __str__(self) -> str:
        return f"Parameter(name='{self.name}', value='{self.value}')"
    
    def __repr__(self) -> str:
        return self.__str__()


class ParameterList:
    """Container for managing parameters."""
    
    def __init__(self):
        """Initialize empty parameter list."""
        self._parameters: Dict[str, Parameter] = {}
    
    def add_parameter(self, name: str, value: Any) -> None:
        """Add parameter to the list.
        
        Args:
            name: Parameter name
            value: Parameter value
        """
        self._parameters[name] = Parameter(name, value)
        logger.debug(f"Added parameter: {name} = {value}")
    
    def get_parameter(self, name: str) -> Parameter:
        """Get parameter by name.
        
        Args:
            name: Parameter name
            
        Returns:
            Parameter instance
            
        Raises:
            ValidationError: If parameter not found
        """
        if name not in self._parameters:
            available = list(self._parameters.keys())
            raise ValidationError(f"Parameter '{name}' not found. Available parameters: {available}")
        
        return self._parameters[name]
    
    def set_parameter(self, name: str, value: Any) -> None:
        """Set parameter value.
        
        Args:
            name: Parameter name
            value: Parameter value
        """
        if name in self._parameters:
            old_value = self._parameters[name].value
            self._parameters[name].value = value
            logger.debug(f"Updated parameter: {name} = {value} (was: {old_value})")
        else:
            self.add_parameter(name, value)
    
    def has_parameter(self, name: str) -> bool:
        """Check if parameter exists.
        
        Args:
            name: Parameter name
            
        Returns:
            True if parameter exists
        """
        return name in self._parameters
    
    def remove_parameter(self, name: str) -> None:
        """Remove parameter from list.
        
        Args:
            name: Parameter name
            
        Raises:
            ValidationError: If parameter not found
        """
        if name not in self._parameters:
            raise ValidationError(f"Parameter '{name}' not found")
        
        del self._parameters[name]
        logger.debug(f"Removed parameter: {name}")
    
    def get_all_parameters(self) -> Dict[str, Parameter]:
        """Get all parameters.
        
        Returns:
            Dictionary of all parameters
        """
        return self._parameters.copy()
    
    def get_parameter_names(self) -> List[str]:
        """Get list of parameter names.
        
        Returns:
            List of parameter names
        """
        return list(self._parameters.keys())
    
    def clear(self) -> None:
        """Clear all parameters."""
        self._parameters.clear()
        logger.debug("Cleared all parameters")
    
    def update_from_dict(self, param_dict: Dict[str, Any]) -> None:
        """Update parameters from dictionary.
        
        Args:
            param_dict: Dictionary of parameter names and values
        """
        for name, value in param_dict.items():
            self.set_parameter(name, value)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary.
        
        Returns:
            Dictionary of parameter names and values
        """
        return {name: param.value for name, param in self._parameters.items()}
    
    def __len__(self) -> int:
        """Get number of parameters."""
        return len(self._parameters)
    
    def __iter__(self):
        """Iterate over parameters."""
        return iter(self._parameters.values())
    
    def __contains__(self, name: str) -> bool:
        """Check if parameter exists."""
        return name in self._parameters
    
    def __str__(self) -> str:
        """String representation."""
        return f"ParameterList({len(self._parameters)} parameters)"
    
    def __repr__(self) -> str:
        return self.__str__()
