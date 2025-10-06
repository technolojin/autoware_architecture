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
from typing import List

from ..models.data_class import ElementData
from ..models.ports import InPort, OutPort
from ..models.links import Link, Connection
from .instances import Instance
from ..parsers.data_parser import element_name_decode

logger = logging.getLogger(__name__)

class DeploymentInstance(Instance):
    def __init__(self, name: str):
        super().__init__(name)

    def set_architecture(
        self,
        architecture:ElementData,
        module_list:List[ElementData],
        pipeline_list:List[ElementData],
        parameter_set_list:List[ElementData],
    ):
        logger.info(f"Setting architecture {architecture.full_name} for instance {self.name}")
        self.element = architecture
        self.element_type = "architecture"

        # 1. set component instances
        logger.info(f"Instance '{self.name}': setting component instances")
        self.set_instances(architecture.full_name, module_list, pipeline_list, parameter_set_list)

        # 2. set connections
        logger.info(f"Instance '{self.name}': setting connections")
        self.set_links()
        self.check_ports()

        # 3. build logical topology
        logger.info(f"Instance '{self.name}': building logical topology")
        # self.build_logical_topology()
        self.set_event_tree()

