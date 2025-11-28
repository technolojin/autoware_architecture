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

class Parameter:
    """Represents a single parameter with its value and metadata."""
    def __init__(self, name: str, value: Any, data_type: str = "string",
                 schema_path: Optional[str] = None, allow_substs: bool = True,
                 is_default: bool = False):
        self.name = name
        self.value = value
        self.data_type = data_type  # string, bool, int, float, etc.
        self.schema_path = schema_path  # path to the schema file if available
        self.allow_substs = allow_substs  # whether to allow substitutions in ROS launch
        self.is_default = is_default  # True if this is a default parameter, False if override

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

    def set_parameter(self, parameter_name, parameter_value, data_type: str = "string",
                     schema_path: Optional[str] = None, allow_substs: bool = True,
                     is_default: bool = False):
        """Set a parameter value.

        Later calls override earlier ones (update in place or append).

        Args:
            parameter_name: Name of the parameter
            parameter_value: Value of the parameter
            data_type: Data type of the value
            schema_path: Optional schema path
            allow_substs: Whether to allow substitutions
            is_default: True if this is a default parameter, False if override
        """
        # Find and update existing parameter, or append new
        for parameter in self.list:
            if parameter.name == parameter_name:
                # Update existing parameter
                parameter.value = parameter_value
                parameter.data_type = data_type
                parameter.schema_path = schema_path
                parameter.allow_substs = allow_substs
                parameter.is_default = is_default
                return
        # Not found, add new parameter
        self.list.append(Parameter(parameter_name, parameter_value, data_type,
                                 schema_path, allow_substs, is_default))

class ParameterFile:
    """Represents a parameter file reference."""
    def __init__(self, name: str, path: str, schema_path: Optional[str] = None,
                 allow_substs: bool = True, is_default: bool = False):
        self.name = name
        self.path = path
        self.schema_path = schema_path  # path to the schema file if available
        self.allow_substs = allow_substs  # whether to allow substitutions in ROS launch
        self.is_default = is_default  # True if this is a default parameter, False if override
class ParameterFileList:
    """Manages an ordered list of parameter files."""

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
                          allow_substs: bool = True, is_default: bool = False):
        """Add a parameter file.

        Simply appends to maintain order. Defaults should be added first,
        then overrides. ROS 2 will handle the actual overriding during loading.

        Args:
            parameter_name: Name of the parameter file
            parameter_path: Path to the parameter file
            schema_path: Optional schema path
            allow_substs: Whether to allow substitutions
            is_default: True if this is a default parameter file, False if override
        """
        new_param_file = ParameterFile(parameter_name, parameter_path, schema_path,
                                     allow_substs, is_default)
        self.list.append(new_param_file)

