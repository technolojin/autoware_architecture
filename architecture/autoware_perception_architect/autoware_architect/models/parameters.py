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
    """Type of parameter - either a file path or a direct configuration value."""
    PARAMETER_FILES = "parameter"  # Load from file
    CONFIGURATION = "configuration"  # Direct overwrite of parameter

class Parameter:
    def __init__(self, name: str, value: str, param_type: ParameterType = ParameterType.CONFIGURATION, 
                 data_type: str = "string", schema_path: Optional[str] = None, 
                 allow_substs: bool = True):
        self.name = name
        self.value = value
        self.param_type = param_type  # PARAMETER_FILES or CONFIGURATION
        self.data_type = data_type  # string, bool, int, float, etc.
        self.schema_path = schema_path  # path to the schema file if available
        self.allow_substs = allow_substs  # whether to allow substitutions in ROS launch
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert parameter to dictionary representation."""
        return {
            "name": self.name,
            "value": self.value,
            "param_type": self.param_type.value,
            "data_type": self.data_type,
            "schema_path": self.schema_path,
            "allow_substs": self.allow_substs
        }

class ParameterList:
    def __init__(self):
        self.list: List[Parameter] = []

    def get_parameter(self, parameter_name):
        for parameter in self.list:
            if parameter.name == parameter_name:
                return parameter.value
        # not found, return None
        return None

    def set_parameter(self, parameter_name, parameter_value, param_type: ParameterType = ParameterType.CONFIGURATION,
                     data_type: str = "string", schema_path: Optional[str] = None, 
                     allow_substs: bool = True):
        for parameter in self.list:
            if parameter.name == parameter_name:
                # if found, overwrite the value and metadata
                parameter.value = parameter_value
                parameter.param_type = param_type
                parameter.data_type = data_type
                parameter.schema_path = schema_path
                parameter.allow_substs = allow_substs
                return
        # if not found, add it
        self.list.append(Parameter(parameter_name, parameter_value, param_type, 
                                 data_type, schema_path, allow_substs))
    
    def get_parameters_dict(self) -> List[Dict[str, Any]]:
        """Get all parameters as list of dictionaries."""
        return [param.to_dict() for param in self.list]
