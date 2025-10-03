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

from .ports import Port, InPort, OutPort

class Link:
    # Link is a connection between two ports
    def __init__(self, msg_type: str, from_port: Port, to_port: Port, namespace: List[str] = []):
        self.msg_type: str = msg_type
        # from-port and to-port connection
        self.from_port: Port = from_port
        self.to_port: Port = to_port
        # namespace
        self.namespace: List[str] = namespace

        self.check_connection()

    def check_connection(self):
        # if the from port is OutPort, it is internal port
        is_from_port_internal = isinstance(self.from_port, OutPort)
        # if the to port is InPort, it is internal port
        is_to_port_internal = isinstance(self.to_port, InPort)

        # case 1: from internal output to internal input
        if is_from_port_internal and is_to_port_internal:
            # propagate and finish the connection
            from_port_list = self.from_port.get_reference_list()
            to_port_list = self.to_port.get_reference_list()

            # check the message type is the same
            for from_port in from_port_list:
                if from_port.msg_type != self.msg_type:
                    raise ValueError(f"Invalid connection: {from_port.name} -> {self.to_port.name}")
            for to_port in to_port_list:
                if to_port.msg_type != self.msg_type:
                    raise ValueError(
                        f"Type mismatch: {self.msg_type}({self.from_port.name}) -> {to_port.msg_type}({to_port.name})"
                    )

            # link the ports
            for from_port_ref in from_port_list:
                from_port_ref.set_users(to_port_list)
            for to_port_ref in to_port_list:
                to_port_ref.set_servers(from_port_list)

            # determine the topic, set it to the from-ports to publish and to-ports to subscribe
            for from_port_ref in from_port_list:
                from_port_ref.set_topic(self.from_port.namespace, self.from_port.name)
            for to_port_ref in to_port_list:
                to_port_ref.set_topic(self.from_port.namespace, self.from_port.name)

            # set the trigger event of the to-port
            for to_port_ref in to_port_list:
                for server_port in to_port_ref.servers:
                    to_port_ref.event.add_trigger_event(server_port.event)

        # case 2: from internal output to external output
        elif is_from_port_internal and not is_to_port_internal:
            # bring the from-port reference to the to-port reference
            reference_port_list = self.from_port.get_reference_list()
            self.to_port.set_references(reference_port_list)
            # set the topic name to the external output, whether it is connected or not
            for reference_port in reference_port_list:
                reference_port.set_topic(self.to_port.namespace, self.to_port.name)

        # case 3: from external input to internal input
        elif not is_from_port_internal and is_to_port_internal:
            # bring the to-port reference to the from-port reference
            reference_port_list = self.to_port.get_reference_list()
            self.from_port.set_references(reference_port_list)

        # case 4: from-port is InPort and to-port is OutPort
        #   bypass connection, which is invalid
        else:
            raise ValueError(f"Invalid connection: {self.from_port.name} -> {self.to_port.name}")


class Connection:
    # Connection is a connection between two elements
    # In other words, it is a configuration to create link(s)
    def __init__(self, connection_dict: dict):

        # connection type
        #   0: undefined
        #   1: external input to internal input
        #   2: internal output to internal input
        #   3: internal output to external output
        self.type = 0

        connection_from = connection_dict.get("from")
        if connection_from is None:
            raise ValueError(f"Connection couldn't found : {connection_dict}")
        connection_to = connection_dict.get("to")
        if connection_to is None:
            raise ValueError(f"Connection couldn't found : {connection_dict}")

        from_instance, from_port_name = self.parse_port_name(connection_from)
        to_instance, to_port_name = self.parse_port_name(connection_to)

        if from_instance == "" and to_instance == "":
            raise ValueError(f"Invalid connection: {connection_dict}")
        elif from_instance == "" and to_instance != "":
            self.type = 1
        elif from_instance != "" and to_instance == "":
            self.type = 3
        elif from_instance != "" and to_instance != "":
            self.type = 2

        self.from_instance: str = from_instance
        self.from_port_name: str = from_port_name
        self.to_instance: str = to_instance
        self.to_port_name: str = to_port_name

    def parse_port_name(self, port_name: str) -> (str, str):  # (instance_name, port_name)
        name_splitted = port_name.split(".")
        if len(name_splitted) == 2:
            if name_splitted[0] == "input":
                return "", name_splitted[1]  # external input
            if name_splitted[0] == "output":
                return "", name_splitted[1]  # external output
            raise ValueError(f"Invalid port name: {port_name}")
        elif len(name_splitted) == 3:
            if name_splitted[1] == "input":
                return name_splitted[0], name_splitted[2]  # internal input
            if name_splitted[1] == "output":
                return name_splitted[0], name_splitted[2]  # internal output
            raise ValueError(f"Invalid port name: {port_name}")
        else:
            raise ValueError(f"Invalid port name: {port_name}")