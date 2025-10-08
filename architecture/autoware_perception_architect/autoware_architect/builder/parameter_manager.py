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

import logging
from typing import TYPE_CHECKING

from ..models.parameters import ParameterList

if TYPE_CHECKING:
    from .instances import Instance

logger = logging.getLogger(__name__)


class ParameterManager:
    """Manages parameter operations for Instance objects."""
    
    def __init__(self, instance: 'Instance'):
        self.instance = instance
        self.parameters: ParameterList = ParameterList()
    
    def set_parameter(self, param):
        """Set parameter based on instance element type."""
        # in case of pipeline, search parameter connection and call set_parameter for children
        if self.instance.element_type == "pipeline":
            self._set_pipeline_parameter(param)
        # in case of module, set the parameter
        elif self.instance.element_type == "module":
            self._set_module_parameter(param)
        else:
            raise ValueError(f"Invalid element type: {self.instance.element_type}")

    def _set_pipeline_parameter(self, param):
        """Set parameter for pipeline element type."""
        if self.instance.element_type != "pipeline":
            raise ValueError("_set_pipeline_parameter is only supported for pipeline")
        param_name = param.get("name")

        # check external_interfaces/parameter
        cfg_pipeline_parameter_list = self.instance.configuration.external_interfaces.get("parameter")
        # check if the param_list_yaml is superset of pipeline_parameter_list
        cfg_pipeline_parameter_list = [param.get("name") for param in cfg_pipeline_parameter_list]
        if param_name not in cfg_pipeline_parameter_list:
            raise ValueError(f"Parameter not found: '{param_name}' in {cfg_pipeline_parameter_list}")

        # check parameters to connect parameters to the children
        cfg_param_connection_list = self.instance.configuration.parameters
        for cfg_connection in cfg_param_connection_list:
            param_from = cfg_connection.get("from")
            param_from_name = param_from.split(".")[1]
            if param_from_name != param_name:
                continue

            param_to = cfg_connection.get("to")
            param_to_inst_name = param_to.split(".")[0]
            child_instance = self.instance.get_child(param_to_inst_name)

            # set the parameter to the child instance
            if child_instance.element_type == "pipeline":
                param["name"] = param_to.split(".")[2]

            child_instance.parameter_manager.set_parameter(param)

    def _set_module_parameter(self, param):
        """Set parameter for module element type."""
        if self.instance.element_type != "module":
            raise ValueError("_set_module_parameter is only supported for module")
        param_path_list = param.get("parameter_paths")
        if not param_path_list:
            raise ValueError(f"No parameter paths found in parameter: {param}")
        # get list of parameter paths, which comes in dictionary format
        for param_path in param_path_list:
            param_keys = param_path.keys()
            for param_key in param_keys:
                param_value = param_path.get(param_key)
                self.parameters.set_parameter(param_key, param_value)
        for param in self.parameters.list:
            logger.debug(f"  Parameter: {param.name} = {param.value}")

    def initialize_module_parameters(self):
        """Initialize parameters for module element during module configuration."""
        if self.instance.element_type != "module":
            return
            
        # set parameters
        for cfg_param in self.instance.configuration.parameters:
            param_name = cfg_param.get("name")
            param_value = cfg_param.get("default")
            self.parameters.set_parameter(param_name, param_value)

    def get_parameter(self, parameter_name: str):
        """Get parameter value by name."""
        return self.parameters.get_parameter(parameter_name)

    def get_all_parameters(self):
        """Get all parameters."""
        return self.parameters.list
