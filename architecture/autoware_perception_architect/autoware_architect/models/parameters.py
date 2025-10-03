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

from typing import List

class Parameter:
    def __init__(self, name: str, value: str):
        self.name = name
        self.value = value

class ParameterList:
    def __init__(self):
        self.list: List[Parameter] = []

    def get_parameter(self, parameter_name):
        for parameter in self.list:
            if parameter.name == parameter_name:
                return parameter.value
        # not found, return None
        return None

    def set_parameter(self, parameter_name, parameter_value):
        for parameter in self.list:
            if parameter.name == parameter_name:
                parameter.value = parameter_value
                return
        # if not found, add it
        self.list.append(Parameter(parameter_name, parameter_value))
