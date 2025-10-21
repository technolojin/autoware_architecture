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
import fnmatch
import re
from typing import List, Dict, TYPE_CHECKING

from ..models.ports import InPort, OutPort
from ..models.links import Link, Connection, ConnectionType

if TYPE_CHECKING:
    from .instances import Instance

logger = logging.getLogger(__name__)


def match_and_pair_wildcard_ports(
    source_pattern: str, 
    target_pattern: str, 
    source_port_names: List[str], 
    target_port_names: List[str]
) -> List[tuple[str, str]]:
    """Match port names against wildcard patterns and return paired port names.
    
    This function matches port names from source and target lists against their respective
    patterns, then pairs them based on wildcard substitution. The pairing is done by
    iterating over the source ports and deriving corresponding target port names.
    
    Args:
        source_pattern: Source port name pattern that may contain wildcards (*, image*, etc.)
        target_pattern: Target port name pattern that may contain wildcards
        source_port_names: List of available source port names to match against
        target_port_names: List of available target port names to match against
        
    Returns:
        List of tuples (source_port_name, target_port_name) that match and can be paired
        
    Examples:
        match_and_pair_wildcard_ports("*", "*", ["image0", "image1"], ["image0", "image1"])
            -> [("image0", "image0"), ("image1", "image1")]
        match_and_pair_wildcard_ports("image*", "camera*", ["image0", "image1"], ["camera0", "camera1"])
            -> [("image0", "camera0"), ("image1", "camera1")]
        match_and_pair_wildcard_ports("filtered_*", "detected_*", ["filtered_obj"], ["detected_obj"])
            -> [("filtered_obj", "detected_obj")]
    """
    # Match source and target ports against their patterns
    if source_pattern == "*":
        matched_source_ports = source_port_names
    else:
        matched_source_ports = [name for name in source_port_names if fnmatch.fnmatch(name, source_pattern)]
    
    if target_pattern == "*":
        matched_target_ports = target_port_names
    else:
        matched_target_ports = [name for name in target_port_names if fnmatch.fnmatch(name, target_pattern)]
    
    # Return empty list if no matches
    if not matched_source_ports or not matched_target_ports:
        return []
    
    # Build pairs by iterating source ports and deriving target names
    pairs = []
    
    for source_name in matched_source_ports:
        # Derive target name using wildcard substitution
        target_name = _apply_wildcard_substitution(source_pattern, target_pattern, source_name)
        
        # Only include the pair if the derived target name exists in matched targets
        if target_name in matched_target_ports:
            pairs.append((source_name, target_name))
    
    return pairs


def _apply_wildcard_substitution(source_pattern: str, target_pattern: str, matched_name: str) -> str:
    """Apply wildcard substitution from source pattern to target pattern.
    
    This is a helper function that extracts the wildcard-matched portion from the source 
    and applies it to the target pattern.
    
    Args:
        source_pattern: The pattern with wildcard used for matching (e.g., "filtered_*")
        target_pattern: The pattern to apply the matched portion to (e.g., "radar_filtered_*")
        matched_name: The actual name that matched the source pattern (e.g., "filtered_objects")
        
    Returns:
        The target name with wildcard substitution applied (e.g., "radar_filtered_objects")
    """
    # If either pattern is just "*", no substitution needed
    if source_pattern == "*" or target_pattern == "*":
        return matched_name
    
    # Convert wildcard pattern to regex pattern
    # Escape special regex characters except *
    source_regex = re.escape(source_pattern).replace(r'\*', '(.*)')
    
    # Match the source pattern against the matched name to extract wildcard content
    match = re.match(f"^{source_regex}$", matched_name)
    if not match:
        # If it doesn't match (shouldn't happen), return the matched name as-is
        return matched_name
    
    # Extract the wildcard-matched portions and apply to target pattern
    wildcard_parts = match.groups()
    result = target_pattern
    for part in wildcard_parts:
        result = result.replace('*', part, 1)  # Replace one * at a time
    
    return result


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
                    f"Message type mismatch: '{port.port_path}' {port.msg_type} != {in_port.msg_type}"
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
                    f"Message type mismatch: '{port.port_path}' {port.msg_type} != {out_port.msg_type}"
                )
            # same port name is found, update reference
            port.set_references(out_port.reference)
            return
        # same port name is not found, add the port
        self.out_ports[out_port.name] = out_port

    def _get_external_interface_names(self, interface_type: str) -> List[str]:
        """Get external interface names (input or output).
        
        Args:
            interface_type: Either "input" or "output"
            
        Returns:
            List of external interface names
        """
        cfg_external_list = self.instance.configuration.external_interfaces.get(interface_type)
        return [ext.get("name") for ext in cfg_external_list]

    def _get_non_global_port_names(self, port_dict: Dict) -> List[str]:
        """Get non-global port names from a port dictionary.
        
        Args:
            port_dict: Dictionary of ports (in_ports or out_ports)
            
        Returns:
            List of non-global port names
        """
        return [name for name, port in port_dict.items() if not port.is_global]

    def _create_link_from_ports(self, from_port, to_port, connection_type: ConnectionType):
        """Create and append a link between two ports.
        
        Args:
            from_port: Source port (InPort or OutPort)
            to_port: Destination port (InPort or OutPort)
            connection_type: Type of connection
        """
        link = Link(from_port.msg_type, from_port, to_port, self.instance.namespace, connection_type)
        self.links.append(link)

    def _create_wildcard_links(self, connection: Connection, 
                              from_port_names: List[str], to_port_names: List[str],
                              from_instance, to_instance):
        """Create links for wildcard connections.
        
        Args:
            connection: Connection configuration
            from_port_names: List of available source port names
            to_port_names: List of available target port names
            from_instance: Source instance (or None for external)
            to_instance: Target instance (or None for external)
        """
        # Match and pair ports based on wildcard patterns
        port_pairs = match_and_pair_wildcard_ports(
            connection.from_port_name, connection.to_port_name,
            from_port_names, to_port_names
        )
        
        # Validate matched ports
        if not port_pairs:
            raise ValueError(
                f"No matching port pairs found: '{connection.from_port_name}' -> '{connection.to_port_name}'"
            )

        # Create links for each matched pair
        for from_port_name, to_port_name in port_pairs:
            # Get or create from_port
            if connection.type == ConnectionType.EXTERNAL_TO_INTERNAL:
                # For external->internal, create InPort based on target's message type
                to_port = to_instance.link_manager.get_in_port(to_port_name)
                from_port = InPort(from_port_name, to_port.msg_type, self.instance.namespace)
            else:
                from_port = from_instance.link_manager.get_out_port(from_port_name)
            
            # Get or create to_port
            if connection.type == ConnectionType.INTERNAL_TO_EXTERNAL:
                to_port = OutPort(to_port_name, from_port.msg_type, self.instance.namespace)
            else:
                to_port = to_instance.link_manager.get_in_port(to_port_name)
            
            self._create_link_from_ports(from_port, to_port, connection.type)

    def _create_external_to_internal_link(self, connection: Connection):
        """Create link from external input to internal input.
        
        Args:
            connection: Connection configuration
        """
        to_instance = self.instance.get_child(connection.to_instance)
        has_wildcard = "*" in connection.from_port_name or "*" in connection.to_port_name
        
        if has_wildcard:
            from_port_names = self._get_external_interface_names("input")
            to_port_names = self._get_non_global_port_names(to_instance.link_manager.in_ports)
            self._create_wildcard_links(connection, from_port_names, to_port_names,
                                       None, to_instance)
        else:
            to_port = to_instance.link_manager.get_in_port(connection.to_port_name)
            from_port = InPort(connection.from_port_name, to_port.msg_type, self.instance.namespace)
            self._create_link_from_ports(from_port, to_port, connection.type)

    def _create_internal_to_internal_link(self, connection: Connection):
        """Create link from internal output to internal input.
        
        Args:
            connection: Connection configuration
        """
        from_instance = self.instance.get_child(connection.from_instance)
        to_instance = self.instance.get_child(connection.to_instance)
        has_wildcard = "*" in connection.from_port_name or "*" in connection.to_port_name
        
        if has_wildcard:
            from_port_names = list(from_instance.link_manager.out_ports.keys())
            to_port_names = list(to_instance.link_manager.in_ports.keys())
            self._create_wildcard_links(connection, from_port_names, to_port_names,
                                       from_instance, to_instance)
        else:
            from_port = from_instance.link_manager.get_out_port(connection.from_port_name)
            to_port = to_instance.link_manager.get_in_port(connection.to_port_name)
            self._create_link_from_ports(from_port, to_port, connection.type)

    def _create_internal_to_external_link(self, connection: Connection):
        """Create link from internal output to external output.
        
        Args:
            connection: Connection configuration
        """
        from_instance = self.instance.get_child(connection.from_instance)
        has_wildcard = "*" in connection.from_port_name or "*" in connection.to_port_name
        
        if has_wildcard:
            from_port_names = self._get_non_global_port_names(from_instance.link_manager.out_ports)
            to_port_names = self._get_external_interface_names("output")
            self._create_wildcard_links(connection, from_port_names, to_port_names,
                                       from_instance, None)
        else:
            from_port = from_instance.link_manager.get_out_port(connection.from_port_name)
            to_port = OutPort(connection.to_port_name, from_port.msg_type, self.instance.namespace)
            self._create_link_from_ports(from_port, to_port, connection.type)

    def set_links(self):
        """Set up links based on element connections."""
        connection_list: List[Connection] = []
        for cfg_connection in self.instance.configuration.connections:
            connection_instance = Connection(cfg_connection)
            connection_list.append(connection_instance)

        # Establish links based on connection type
        for connection in connection_list:
            if connection.type == ConnectionType.EXTERNAL_TO_INTERNAL:
                self._create_external_to_internal_link(connection)
            elif connection.type == ConnectionType.INTERNAL_TO_INTERNAL:
                self._create_internal_to_internal_link(connection)
            elif connection.type == ConnectionType.INTERNAL_TO_EXTERNAL:
                self._create_internal_to_external_link(connection)

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
            logger.debug(f"  In port: {in_port.port_path}")
            logger.debug(f"    Subscribing topic: {in_port.topic}")
            server_port_list = in_port.servers
            if server_port_list == []:
                logger.debug("    Server port not found")
                continue
            for server_port in server_port_list:
                logger.debug(f"    server: {server_port.port_path}, topic: {server_port.topic}")

        for out_port in self.out_ports.values():
            logger.debug(f"  Out port: {out_port.port_path}")
            user_port_list = out_port.users
            if user_port_list == []:
                logger.debug("    User port not found")
                continue
            for user_port in user_port_list:
                logger.debug(f"    user: {user_port.port_path}")

    def log_pipeline_configuration(self):
        """Log pipeline configuration details."""
        logger.debug(
            f"Instance '{self.instance.name}' pipeline configuration: {len(self.links)} links established"
        )
        for link in self.links:
            logger.debug(f"  Link: {link.from_port.port_path} -> {link.to_port.port_path}")
        # new ports
        for in_port in self.in_ports.values():
            logger.debug(f"  New in port: {in_port.port_path}")
        for out_port in self.out_ports.values():
            logger.debug(f"  New out port: {out_port.port_path}")

    def get_all_in_ports(self):
        """Get all input ports."""
        return list(self.in_ports.values())

    def get_all_out_ports(self):
        """Get all output ports."""
        return list(self.out_ports.values())

    def get_all_links(self):
        """Get all links."""
        return self.links
