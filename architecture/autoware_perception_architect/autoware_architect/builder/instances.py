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
from typing import List, Dict

from ..models.data_class import ModuleElement, PipelineElement, ParameterSetElement, ArchitectureElement
from ..models.events import Event, Process
from ..parsers.data_parser import element_name_decode
from ..config import config
from .parameter_manager import ParameterManager
from .link_manager import LinkManager

logger = logging.getLogger(__name__)

class Instance:
    # Common attributes for node hierarch instance
    def __init__(
        self, name: str, compute_unit: str = "", namespace: list[str] = [], layer: int = 0
    ):
        self.name: str = name
        self.namespace: List[str] = namespace.copy()
        # add the instance name to the namespace
        self.namespace.append(name)
        # create namespace string, FOR ERROR MESSAGE ONLY
        self.namespace_str: str = "/" + "/".join(self.namespace)
        self.id = ("__".join(self.namespace) + "__" + name).replace("/", "__")

        self.compute_unit: str = compute_unit
        self.layer: int = layer
        if self.layer > config.layer_limit:
            raise ValueError(f"Instance layer is too deep (limit: {config.layer_limit})")

        # element
        self.element: ModuleElement | PipelineElement | ParameterSetElement | ArchitectureElement | None = None
        self.element_type: str = None
        self.parent: Instance = None
        self.children: Dict[str, Instance] = {}
        self.parent_pipeline_list: List[str] = []

        # interface
        self.link_manager: LinkManager = LinkManager(self)

        # processes
        self.processes: List[Process] = []
        self.event_list: List[Event] = []

        # parameter manager
        self.parameter_manager: ParameterManager = ParameterManager(self)

        # status
        self.is_initialized = False

    def set_instances(self, element_id, module_list, pipeline_list, parameter_set_list):
        element_name, element_type = element_name_decode(element_id)
        if element_type == "architecture":
            for component in self.element.components:
                compute_unit_name = component.get("compute_unit")
                instance_name = component.get("component")
                element_id = component.get("element")
                namespace = component.get("namespace")

                # create instance
                instance = Instance(instance_name, compute_unit_name, [namespace])
                instance.parent = self
                try:
                    instance.set_instances(element_id, module_list, pipeline_list, parameter_set_list)
                except Exception as e:
                    # add the instance to the children dict for debugging
                    self.children[instance_name] = instance
                    raise ValueError(f"Error in setting component instance '{instance_name}' : {e}")

                # parameter set
                parameter_set = component.get("parameter_set")
                param_list_yaml = None
                if parameter_set is not None:
                    param_set_name, element_type = element_name_decode(parameter_set)
                    if element_type != "parameter_set":
                        raise ValueError(f"Invalid parameter set type: {element_type}")
                    param_set = parameter_set_list.get(param_set_name)
                    param_list_yaml = param_set.config.get("parameters")

                if param_list_yaml is not None:
                    for param in param_list_yaml:
                        instance.parameter_manager.set_parameter(param)

                self.children[instance_name] = instance
            # all children are initialized
            self.is_initialized = True

        elif element_type == "pipeline":
            logger.info(f"Setting pipeline element {element_id} for instance {self.namespace_str}")
            self.element = pipeline_list.get(element_name)
            self.element_type = element_type

            # check if the pipeline is already set
            if element_id in self.parent_pipeline_list:
                raise ValueError(f"Element is already set: {element_id}, avoid circular reference")
            self.parent_pipeline_list.append(element_id)

            # set children
            node_list = self.element.nodes
            for node in node_list:
                instance = Instance(
                    node.get("node"), self.compute_unit, self.namespace, self.layer + 1
                )
                instance.parent = self
                instance.parent_pipeline_list = self.parent_pipeline_list.copy()
                # recursive call of set_instances
                try:
                    instance.set_instances(node.get("element"), module_list, pipeline_list, parameter_set_list)
                except Exception as e:
                    # add the instance to the children dict for debugging
                    self.children[instance.name] = instance
                    raise ValueError(f"Error in setting child instance {instance.name} : {e}")
                self.children[instance.name] = instance

            # run the pipeline configuration
            self._run_pipeline_configuration()

            # recursive call is finished
            self.is_initialized = True

        elif element_type == "module":
            logger.info(f"Setting module element {element_id} for instance {self.namespace_str}")
            self.element = module_list.get(element_name)
            self.element_type = element_type

            # run the module configuration
            self._run_module_configuration()

            # recursive call is finished
            self.is_initialized = True

        else:
            raise ValueError(f"Invalid element type: {element_type}")
        
    def _run_pipeline_configuration(self):
        if self.element_type != "pipeline":
            raise ValueError("run_pipeline_configuration is only supported for pipeline")

        # set connections
        if len(self.element.connections) == 0:
            raise ValueError("No connections found in the pipeline configuration")

        self.link_manager.set_links()

        # create external ports
        self.link_manager.create_external_ports(self.link_manager.links)

        # log pipeline configuration
        self.link_manager.log_pipeline_configuration()

    def _run_module_configuration(self):
        if self.element_type != "module":
            raise ValueError("run_module_configuration is only supported for module")

        # set ports
        self.link_manager.initialize_module_ports()

        # set parameters
        self.parameter_manager.initialize_module_parameters()

        # connect port events and the process events
        on_input_events = self.link_manager.get_input_events()
        to_output_events = self.link_manager.get_output_events()

        # parse processes and get trigger conditions and output conditions
        for process_config in self.element.processes:
            name = process_config.get("name")
            self.processes.append(Process(name, self.namespace, process_config))

        # set the process events
        process_event_list = [process.event for process in self.processes]
        if len(process_event_list) == 0:
            # process configuration is not found
            raise ValueError(f"No process found in {self.name}")
        for process in self.processes:
            process.set_condition(process_event_list, on_input_events)
            process.set_outcomes(process_event_list, to_output_events)

        # set the process events
        process_event_list = []
        for process in self.processes:
            process_event_list.extend(process.get_event_list())
        self.event_list = process_event_list

    def get_child(self, name: str):
        if name in self.children:
            return self.children[name]
        raise ValueError(f"Child not found: child name '{name}', instance of '{self.name}'")

    def get_in_port(self, name: str):
        return self.link_manager.get_in_port(name)

    def get_out_port(self, name: str):
        return self.link_manager.get_out_port(name)

    def check_ports(self):
        # recursive call for children
        for child in self.children.values():
            child.check_ports()

        # delegate to link manager
        self.link_manager.check_ports()

    def set_event_tree(self):
        # trigger the event tree from the current instance
        # in case of pipeline, event_list is empty
        for event in self.event_list:
            event.set_frequency_tree()
        # recursive call for children
        # in case of module, children is empty
        for child in self.children.values():
            child.set_event_tree()

    def collect_instance_data(self):
        data = {
            "name": self.name,
            "id": self.id,
            "element_type": self.element_type,
            "namespace": self.namespace,
            "compute_unit": self.compute_unit,
            "in_ports": self.link_manager.get_all_in_ports(),
            "out_ports": self.link_manager.get_all_out_ports(),
            "children": (
                [child.collect_instance_data() for child in self.children.values()]
                if hasattr(self, "children")
                else []
            ),
            "links": (
                [
                    {
                        "from_port": link.from_port,
                        "to_port": link.to_port,
                        "msg_type": link.msg_type,
                    }
                    for link in self.link_manager.get_all_links()
                ]
                if hasattr(self.link_manager, "links")
                else []
            ),
            "events": (self.event_list),
            "parameters": self.parameter_manager.get_all_parameters(),
        }

        return data

