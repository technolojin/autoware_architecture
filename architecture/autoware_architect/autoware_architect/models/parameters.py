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
    """Parameter type with priority ordering (lower value = lower priority)."""
    DEFAULT = 1          # Default parameter
    DEFAULT_FILE = 2     # Default loaded from parameter file
    OVERRIDE_FILE = 3    # Override loaded from parameter file
    OVERRIDE = 4         # Override parameter

class Parameter:
    """Represents a single parameter with its value and metadata."""
    def __init__(self, name: str, value: Any, data_type: str = "string",
                 schema_path: Optional[str] = None, allow_substs: bool = True,
                 parameter_type: ParameterType = ParameterType.DEFAULT):
        self.name = name
        self.value = value
        self.data_type = data_type  # string, bool, int, float, etc.
        self.schema_path = schema_path  # path to the schema file if available
        self.allow_substs = allow_substs  # whether to allow substitutions in ROS launch
        self.parameter_type = parameter_type  # Parameter type with priority

class ParameterList:
    """Manages an ordered list of parameters, maintaining priority order (higher priority comes later)."""

    def __init__(self):
        self.list: List[Parameter] = []

    def get_parameter(self, parameter_name):
        """Get the last (most recent/override) parameter value by name."""
        for parameter in reversed(self.list):
            if parameter.name == parameter_name:
                return parameter.value
        # not found, return None
        return None

    def set_parameter(self, parameter_name, parameter_value, data_type: str = "string",
                     schema_path: Optional[str] = None, allow_substs: bool = True,
                     parameter_type: ParameterType = ParameterType.DEFAULT):
        """Set a parameter value.

        Higher priority parameters override lower priority ones.
        Parameters are maintained in priority order (lower to higher).

        Args:
            parameter_name: Name of the parameter
            parameter_value: Value of the parameter
            data_type: Data type of the value
            schema_path: Optional schema path
            allow_substs: Whether to allow substitutions
            parameter_type: Type of parameter with priority
        """
        # Find and update existing parameter, or append new
        for parameter in self.list:
            if parameter.name == parameter_name:
                # Update existing parameter
                parameter.value = parameter_value
                parameter.data_type = data_type
                parameter.schema_path = schema_path
                parameter.allow_substs = allow_substs
                parameter.parameter_type = parameter_type
                return
        # Not found, add new parameter
        self.list.append(Parameter(parameter_name, parameter_value, data_type,
                                 schema_path, allow_substs, parameter_type))

class ParameterFile:
    """Represents a parameter file reference."""
    def __init__(self, name: str, path: str, schema_path: Optional[str] = None,
                 allow_substs: bool = True, parameter_type: ParameterType = ParameterType.DEFAULT_FILE):
        self.name = name
        self.path = path
        self.schema_path = schema_path  # path to the schema file if available
        self.allow_substs = allow_substs  # whether to allow substitutions in ROS launch
        self.parameter_type = parameter_type  # Parameter type with priority
class ParameterFileList:
    """Manages an ordered list of parameter files by priority."""

    def __init__(self):
        self.list: List[ParameterFile] = []

    def get_parameter_file(self, parameter_name):
        """Get the last (most recent/override) parameter file path by name."""
        for param_file in reversed(self.list):
            if param_file.name == parameter_name:
                return param_file.path
        # not found, return None
        return None

    def add_parameter_file(self, parameter_name, parameter_path, schema_path: Optional[str] = None,
                          allow_substs: bool = True, parameter_type: ParameterType = ParameterType.DEFAULT_FILE):
        """Add a parameter file.

        Higher priority parameter files override lower priority ones.
        Parameter files are maintained in priority order.

        Args:
            parameter_name: Name of the parameter file
            parameter_path: Path to the parameter file
            schema_path: Optional schema path
            allow_substs: Whether to allow substitutions
            parameter_type: Type of parameter file with priority
        """
        new_param_file = ParameterFile(parameter_name, parameter_path, schema_path,
                                     allow_substs, parameter_type)
        self.list.append(new_param_file)

