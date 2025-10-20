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
import logging
from .events import Event

logger = logging.getLogger(__name__)

def generate_port_path(namespace: List[str], name: str) -> str:
    if namespace:
        return "/" + "/".join(namespace) + "/" + name
    return "/" + name

class Port:
    def __init__(self, name: str, msg_type: str, namespace: List[str] = []):
        self.name = name
        self.msg_type = msg_type
        self.namespace = namespace
        self.port_path = generate_port_path(namespace, name)
        self.reference: List["Port"] = []
        self.topic: List[str] = []
        self.event = None
        self.is_global = False

    @property
    def unique_id(self):
        # Remove leading slash, replace remaining slashes with double underscores
        return self.port_path.lstrip("/").replace("/", "__")

    def set_references(self, port_list: List["Port"]):
        reference_name_list = [p.port_path for p in self.reference]
        added = []
        for port in port_list:
            if port.port_path not in reference_name_list:
                self.reference.append(port)
                added.append(port.port_path)
        if added:
            logger.debug(f"Port '{self.port_path}' added references: {added}")

    def get_reference_list(self):
        if self.reference == []:
            return [self]
        return self.reference

    def set_topic(self, topic_namespace: List[str], topic_name: str):
        self.topic = topic_namespace + [topic_name]


class InPort(Port):
    def __init__(self, name, msg_type, namespace: List[str] = []):
        super().__init__(name, msg_type, namespace)
        self.port_path = generate_port_path(namespace, "input_" + name)
        self.is_required = True
        self.servers: List[Port] = []
        self.event = Event("input_" + name, namespace)
        self.event.set_type("on_input")

    def set_servers(self, port_list: List[Port]):
        server_name_list = [p.port_path for p in self.servers]
        added = []
        for port in port_list:
            if port.port_path not in server_name_list:
                self.servers.append(port)
                added.append(port.port_path)
        if added:
            logger.debug(f"InPort '{self.port_path}' added servers: {added}")


class OutPort(Port):
    def __init__(self, name, msg_type, namespace: List[str] = []):
        super().__init__(name, msg_type, namespace)
        self.port_path = generate_port_path(namespace, "output_" + name)
        self.frequency = 0.0
        self.is_monitored = False
        self.users: List[Port] = []
        self.event = Event("output_" + name, namespace)
        self.event.set_type("to_output")

    def set_users(self, port_list: List[Port]):
        user_name_list = [p.port_path for p in self.users]
        added = []
        for port in port_list:
            if port.port_path not in user_name_list:
                self.users.append(port)
                added.append(port.port_path)
        if added:
            logger.debug(f"OutPort '{self.port_path}' added users: {added}")
