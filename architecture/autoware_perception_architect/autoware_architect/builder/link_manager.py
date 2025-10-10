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
from typing import List, Dict, TYPE_CHECKING

from ..models.ports import InPort, OutPort
from ..models.links import Link, Connection, ConnectionType

if TYPE_CHECKING:
    from .instances import Instance

logger = logging.getLogger(__name__)


class LinkManager:
    """Manages port, connection, and link operations for Instance objects."""
    
    def __init__(self, instance: 'Instance'):
        self.instance = instance
        
        # interface
        self.in_ports: Dict[str, InPort] = {}
        self.out_ports: Dict[str, OutPort] = {}
        self.links: List[Link] = []

    def get_in_port(self, name: str):
        """Get input port by name."""
        if name in self.in_ports:
            return self.in_ports[name]
        raise ValueError(f"In port not found: in-port name '{name}', instance of '{self.instance.name}'")

    def get_out_port(self, name: str):
        """Get output port by name."""
        if name in self.out_ports:
            return self.out_ports[name]
        raise ValueError(f"Out port not found: out-port name '{name}', instance of '{self.instance.name}'")

    def set_in_port(self, in_port: InPort):
        """Set input port after validation."""
        # check the external input is defined
        cfg_external_input_list = self.instance.configuration.external_interfaces.get("input")
        cfg_external_input_list = [ext_input.get("name") for ext_input in cfg_external_input_list]
        if in_port.name not in cfg_external_input_list:
            raise ValueError(
                f"External input not found: '{in_port.name}' in '{cfg_external_input_list}'"
            )

        # check if there is a port with the same name
        if in_port.name in self.in_ports:
            port = self.in_ports[in_port.name]
            # check if the message type is the same
            if port.msg_type != in_port.msg_type:
                raise ValueError(
                    f"Message type mismatch: '{port.full_name}' {port.msg_type} != {in_port.msg_type}"
                )
            # same port name is found, update reference
            port.set_references(in_port.reference)
            return
        # same port name is not found, add the port
        self.in_ports[in_port.name] = in_port

    def set_out_port(self, out_port: OutPort):
        """Set output port after validation."""
        # check the external output is defined
        cfg_external_output_list = self.instance.configuration.external_interfaces.get("output")
        cfg_external_output_list = [ext_output.get("name") for ext_output in cfg_external_output_list]
        if out_port.name not in cfg_external_output_list:
            raise ValueError(
                f"External output not found: '{out_port.name}' in {cfg_external_output_list}"
            )

        # check if there is a port with the same name
        if out_port.name in self.out_ports:
            port = self.out_ports[out_port.name]
            # check if the message type is the same
            if port.msg_type != out_port.msg_type:
                raise ValueError(
                    f"Message type mismatch: '{port.full_name}' {port.msg_type} != {out_port.msg_type}"
                )
            # same port name is found, update reference
            port.set_references(out_port.reference)
            return
        # same port name is not found, add the port
        self.out_ports[out_port.name] = out_port

    def set_links(self):
        """Set up links based on element connections."""
        connection_list: List[Connection] = []
        for cfg_connection in self.instance.configuration.connections:
            connection_instance = Connection(cfg_connection)
            connection_list.append(connection_instance)

        # establish links
        for connection in connection_list:
            # case 1. from external input to internal input
            if connection.type == ConnectionType.EXTERNAL_TO_INTERNAL:
                # find the to_instance from children
                to_instance = self.instance.get_child(connection.to_instance)
                port_list = list(to_instance.link_manager.in_ports.values())
                if len(port_list) == 0:
                    raise ValueError(f"No available port found in {to_instance.name}")
                # if the port name is wildcard, find available port from the to_instance
                if connection.to_port_name == "*":
                    for port in port_list:
                        from_port = InPort(port.name, port.msg_type, self.instance.namespace)
                        link = Link(port.msg_type, from_port, port, self.instance.namespace, connection.type)
                        self.links.append(link)
                else:
                    # match the port name
                    to_port = to_instance.link_manager.get_in_port(connection.to_port_name)
                    # create a link
                    from_port = InPort(
                        connection.from_port_name, to_port.msg_type, self.instance.namespace
                    )
                    link = Link(to_port.msg_type, from_port, to_port, self.instance.namespace, connection.type)
                    self.links.append(link)

            # case 2. from internal output to internal input
            if connection.type == ConnectionType.INTERNAL_TO_INTERNAL:
                # find the from_instance and to_instance from children
                from_instance = self.instance.get_child(connection.from_instance)
                to_instance = self.instance.get_child(connection.to_instance)
                # find the from_port and to_port
                from_port = from_instance.link_manager.get_out_port(connection.from_port_name)
                to_port = to_instance.link_manager.get_in_port(connection.to_port_name)
                # create link
                link = Link(from_port.msg_type, from_port, to_port, self.instance.namespace, connection.type)
                self.links.append(link)

            # case 3. from internal output to external output
            if connection.type == ConnectionType.INTERNAL_TO_EXTERNAL:
                # find the from_instance from children
                from_instance = self.instance.get_child(connection.from_instance)
                port_list = list(from_instance.link_manager.out_ports.values())
                if len(port_list) == 0:
                    raise ValueError(f"No available port found in {from_instance.name}")
                # if the port name is wildcard, find available port from the from_instance
                if connection.from_port_name == "*":
                    for port in port_list:
                        to_port = OutPort(port.name, port.msg_type, self.instance.namespace)
                        link = Link(port.msg_type, port, to_port, self.instance.namespace, connection.type)
                        self.links.append(link)
                else:
                    # match the port name
                    from_port = from_instance.link_manager.get_out_port(connection.from_port_name)
                    # create link
                    to_port = OutPort(
                        connection.to_port_name, from_port.msg_type, self.instance.namespace
                    )
                    link = Link(from_port.msg_type, from_port, to_port, self.instance.namespace, connection.type)
                    self.links.append(link)

    def create_external_ports(self, link_list):
        """Create external ports based on link list."""
        # create in ports based on the link_list
        for link in link_list:
            # create port only if the namespace is the same as the instance
            if link.from_port.namespace == self.instance.namespace:
                # set the in_port
                self.set_in_port(link.from_port)
            if link.to_port.namespace == self.instance.namespace:
                # set the out_port
                self.set_out_port(link.to_port)

    def initialize_module_ports(self):
        """Initialize ports for module element during module configuration."""
        if self.instance.element_type != "module":
            return
            
        # set in_ports
        for cfg_in_port in self.instance.configuration.inputs:
            in_port_name = cfg_in_port.get("name")
            in_port_msg_type = cfg_in_port.get("message_type")
            in_port_instance = InPort(in_port_name, in_port_msg_type, self.instance.namespace)
            if "global" in cfg_in_port:
                in_port_instance.is_global = True
                topic = cfg_in_port.get("global")
                if topic[0] == "/":
                    topic = topic[1:]
                in_port_instance.topic = topic.split("/")
            self.in_ports[in_port_name] = in_port_instance

        # set out_ports
        for cfg_out_port in self.instance.configuration.outputs:
            out_port_name = cfg_out_port.get("name")
            out_port_msg_type = cfg_out_port.get("message_type")
            out_port_instance = OutPort(out_port_name, out_port_msg_type, self.instance.namespace)
            if "global" in cfg_out_port:
                out_port_instance.is_global = True
                topic = cfg_out_port.get("global")
                if topic[0] == "/":
                    topic = topic[1:]
                out_port_instance.topic = topic.split("/")
            self.out_ports[out_port_name] = out_port_instance

    def get_input_events(self):
        """Get input events from all input ports."""
        return [in_port.event for in_port in self.in_ports.values()]

    def get_output_events(self):
        """Get output events from all output ports."""
        return [out_port.event for out_port in self.out_ports.values()]

    def check_ports(self):
        """Check and debug port configurations."""
        # check ports only for module. in case of pipeline, the check is done
        if self.instance.element_type != "module":
            return

        # check ports
        for in_port in self.in_ports.values():
            logger.debug(f"  In port: {in_port.full_name}")
            logger.debug(f"    Subscribing topic: {in_port.topic}")
            server_port_list = in_port.servers
            if server_port_list == []:
                logger.debug("    Server port not found")
                continue
            for server_port in server_port_list:
                logger.debug(f"    server: {server_port.full_name}, topic: {server_port.topic}")

        for out_port in self.out_ports.values():
            logger.debug(f"  Out port: {out_port.full_name}")
            user_port_list = out_port.users
            if user_port_list == []:
                logger.debug("    User port not found")
                continue
            for user_port in user_port_list:
                logger.debug(f"    user: {user_port.full_name}")

    def log_pipeline_configuration(self):
        """Log pipeline configuration details."""
        logger.debug(
            f"Instance '{self.instance.name}' pipeline configuration: {len(self.links)} links established"
        )
        for link in self.links:
            logger.debug(f"  Link: {link.from_port.full_name} -> {link.to_port.full_name}")
        # new ports
        for in_port in self.in_ports.values():
            logger.debug(f"  New in port: {in_port.full_name}")
        for out_port in self.out_ports.values():
            logger.debug(f"  New out port: {out_port.full_name}")

    def get_all_in_ports(self):
        """Get all input ports."""
        return list(self.in_ports.values())

    def get_all_out_ports(self):
        """Get all output ports."""
        return list(self.out_ports.values())

    def get_all_links(self):
        """Get all links."""
        return self.links
