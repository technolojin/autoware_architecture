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

from ..models.data_class import ElementData

from ..models.ports import InPort, OutPort
from ..models.links import Link, Connection
from .instances import Instance

from ..parsers.data_parser import element_name_decode

debug_mode = False

class DeploymentInstance(Instance):
    def __init__(self, name: str):
        super().__init__(name)

    def set_component_instances(self, module_list, pipeline_list, parameter_set_list):
        # set pipeline and module instances as 'components'
        for component in self.element.config.get("components"):
            compute_unit_name = component.get("compute_unit")

            instance_name = component.get("component")
            element_id = component.get("element")
            namespace = component.get("namespace")

            # parameter set
            parameter_set = component.get("parameter_set")
            param_list_yaml = None
            if parameter_set is not None:
                param_set_name, element_type = element_name_decode(parameter_set)
                if element_type != "parameter_set":
                    raise ValueError(f"Invalid parameter set type: {element_type}")
                param_set = parameter_set_list.get(param_set_name)
                param_list_yaml = param_set.config.get("parameters")

            # create instance
            instance = Instance(instance_name, compute_unit_name, [namespace])
            instance.parent = self
            try:
                instance.set_element(element_id, module_list, pipeline_list)
            except Exception as e:
                # add the instance to the children list for debugging
                self.children.append(instance)
                raise ValueError(f"Error in setting component instance '{instance_name}' : {e}")

            if param_list_yaml is not None:
                for param in param_list_yaml:
                    instance.set_parameter(param)

            self.children.append(instance)
        # all children are initialized
        self.is_initialized = True

    def set_architecture(
        self,
        architecture:ElementData,
        module_list:List[ElementData],
        pipeline_list:List[ElementData],
        parameter_set_list:List[ElementData],
    ):
        if debug_mode:
            print(
                f"Instance set_architecture: Setting {architecture.full_name} instance {self.name}"
            )
        self.element = architecture
        self.element_type = "architecture"

        # set component instances
        self.set_component_instances(module_list, pipeline_list, parameter_set_list)

    def set_connections(self):
        # 2. connect instances
        # set connections
        connection_list_yaml = self.element.config.get("connections")
        if len(connection_list_yaml) == 0:
            raise ValueError("No connections found in the pipeline configuration")

        connection_list: List[Connection] = []
        for connection in connection_list_yaml:
            connection_instance = Connection(connection)
            connection_list.append(connection_instance)

        # establish links. topics will be defined in this step
        link_list: List[Link] = []
        for connection in connection_list:
            # find the from_instance and to_instance from children
            from_instance = self.get_child(connection.from_instance)
            to_instance = self.get_child(connection.to_instance)
            # find the from_port and to_port
            from_port = from_instance.get_out_port(connection.from_port_name)
            to_port = to_instance.get_in_port(connection.to_port_name)
            # check if the port type
            if not isinstance(from_port, OutPort):
                raise ValueError(f"Invalid port type: {from_port.full_name}")
            if not isinstance(to_port, InPort):
                raise ValueError(f"Invalid port type: {to_port.full_name}")
            # create link
            link = Link(from_port.msg_type, from_port, to_port, self.namespace)
            link_list.append(link)

        for link in link_list:
            self.links.append(link)
            if debug_mode:
                print(f"Connection: {link.from_port.full_name}-> {link.to_port.full_name}")
        if debug_mode:
            print(f"Instance {self.name} set_architecture: {len(self.links)} links are established")
            for link in self.links:
                print(f"  Link: {link.from_port.full_name} -> {link.to_port.full_name}")

        # check ports
        if debug_mode:
            print(f"\n\nInstance '{self.name}': checking ports")
        self.check_ports()

    def build_logical_topology(self):
        print(f"\nInstance '{self.name}': building logical topology")
        # build logical topology
        for child in self.children:
            child.set_event_tree()


