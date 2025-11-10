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

from typing import List, Optional, Dict, Any
from enum import Enum

class ParameterType(Enum):
    """Type of parameter - either a file path or a direct parameter value."""
    PARAMETER_FILE = "parameter"  # Load from file
    PARAMETER = "parameter"  # Direct overwrite of parameter

class Parameter:
    def __init__(self, name: str, value: str, param_type: ParameterType = ParameterType.PARAMETER, 
                 data_type: str = "string", schema_path: Optional[str] = None, 
                 allow_substs: bool = True, is_default: bool = False):
        self.name = name
        self.value = value
        self.param_type = param_type  # PARAMETER_FILE or PARAMETER
        self.data_type = data_type  # string, bool, int, float, etc.
        self.schema_path = schema_path  # path to the schema file if available
        self.allow_substs = allow_substs  # whether to allow substitutions in ROS launch
        self.is_default = is_default  # True if this is a default parameter, False if override
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert parameter to dictionary representation."""
        return {
            "name": self.name,
            "value": self.value,
            "param_type": self.param_type.value,
            "data_type": self.data_type,
            "schema_path": self.schema_path,
            "allow_substs": self.allow_substs,
            "is_default": self.is_default
        }

class ParameterList:
    """Manages an ordered list of parameters, maintaining order with defaults first and overrides later."""
    
    def __init__(self):
        self.list: List[Parameter] = []

    def get_parameter(self, parameter_name):
        """Get the last (most recent/override) parameter value by name."""
        for parameter in reversed(self.list):
            if parameter.name == parameter_name:
                return parameter.value
        # not found, return None
        return None

    def set_parameter(self, parameter_name, parameter_value, param_type: ParameterType = ParameterType.PARAMETER,
                     data_type: str = "string", schema_path: Optional[str] = None, 
                     allow_substs: bool = True, is_default: bool = False):
        """Set a parameter value.
        
        For parameters: Later calls override earlier ones (update in place or append).
        For parameter_files: Simply appends to maintain order. Defaults should be added first,
                           then overrides. ROS 2 will handle the actual overriding during loading.
        
        Args:
            parameter_name: Name of the parameter
            parameter_value: Value of the parameter
            param_type: Type of parameter (PARAMETER_FILE or PARAMETER)
            data_type: Data type of the value
            schema_path: Optional schema path
            allow_substs: Whether to allow substitutions
            is_default: True if this is a default parameter, False if override
        """
        if param_type == ParameterType.PARAMETER:
            # For parameters: find and update existing, or append new
            for parameter in self.list:
                if parameter.name == parameter_name and parameter.param_type == ParameterType.PARAMETER:
                    # Update existing parameter
                    parameter.value = parameter_value
                    parameter.data_type = data_type
                    parameter.schema_path = schema_path
                    parameter.allow_substs = allow_substs
                    parameter.is_default = is_default
                    return
            # Not found, add new parameter
            self.list.append(Parameter(parameter_name, parameter_value, param_type, 
                                     data_type, schema_path, allow_substs, is_default))
        else:
            # For parameter_files: simply append in order
            # ROS 2 parameter loader will handle overriding - later files override earlier ones
            # Caller is responsible for adding defaults first, then overrides
            new_param = Parameter(parameter_name, parameter_value, param_type, 
                                data_type, schema_path, allow_substs, is_default)
            self.list.append(new_param)
    
    def get_parameters_dict(self) -> List[Dict[str, Any]]:
        """Get all parameters as list of dictionaries in order."""
        return [param.to_dict() for param in self.list]
    
    def get_parameter_files_ordered(self) -> List[Parameter]:
        """Get all parameter files in order (defaults first, then overrides).
        
        This ensures that even if parameters were added out of order,
        defaults are always loaded before overrides in the launcher.
        """
        param_files = [param for param in self.list if param.param_type == ParameterType.PARAMETER_FILE]
        # Sort: defaults (is_default=True) first, then overrides (is_default=False)
        # Stable sort preserves relative order within each group
        return sorted(param_files, key=lambda p: (not p.is_default))
    
    def get_parameters(self) -> List[Parameter]:
        """Get all parameters."""
        return [param for param in self.list if param.param_type == ParameterType.PARAMETER]

