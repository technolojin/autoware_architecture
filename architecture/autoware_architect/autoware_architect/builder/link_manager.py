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
from typing import Any, Dict, List, TYPE_CHECKING

from ..models.ports import InPort, OutPort
from ..models.links import Link, Connection, ConnectionType

if TYPE_CHECKING:
    from .instances import Instance

logger = logging.getLogger(__name__)


def match_and_pair_wildcard_ports(
    source_pattern: str,
    target_pattern: str,
    source_ports: Dict[str, Any],
    target_ports: Dict[str, Any],
) -> List[tuple[str, str]]:
    """Match port identifiers against wildcard patterns and return paired keys.

    This helper takes dictionaries of available ports and matches their keys against the
    provided source/target patterns. When a source key matches the source pattern, the
    wildcard fragment is substituted into the target pattern to derive the desired
    target key. Pairs are returned only when the derived target key exists in the
    target dictionary.

    Args:
        source_pattern: Source port key pattern that may contain wildcards (*, image*, etc.)
        target_pattern: Target port key pattern that may contain wildcards
        source_ports: Mapping of available source port keys to their metadata
        target_ports: Mapping of available target port keys to their metadata

    Returns:
        List of tuples ``(source_key, target_key)`` that satisfy the wildcard pairing

    Examples:
        >>> match_and_pair_wildcard_ports(
        ...     "module.image*",
        ...     "module.camera*",
        ...     {"module.image0": {}},
        ...     {"module.camera0": {}},
        ... )
        [("module.image0", "module.camera0")]
    """

    def _matched_keys(pattern: str, keys: List[str]) -> List[str]:
        if pattern == "*":
            return keys
        return [key for key in keys if fnmatch.fnmatch(key, pattern)]

    source_keys = list(source_ports.keys())
    target_keys = list(target_ports.keys())

    matched_source_keys = _matched_keys(source_pattern, source_keys)
    matched_target_keys = _matched_keys(target_pattern, target_keys)

    if not matched_source_keys or not matched_target_keys:
        return []

    pairs: List[tuple[str, str]] = []

    if source_pattern == "*" and target_pattern == "*":
        for name in matched_source_keys:
            if name in matched_target_keys:
                pairs.append((name, name))
        return pairs

    for source_key in matched_source_keys:
        target_key = _apply_wildcard_substitution(source_pattern, target_pattern, source_key)
        if target_key in matched_target_keys:
            if source_pattern == "*" and target_pattern == "*":
                pairs.append((source_key, target_key))
            elif source_key != target_key:
                pairs.append((source_key, target_key))

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

    def _resolve_ports_for_connection(
        self,
        connection: Connection,
        from_info: Dict[str, Any] | None,
        to_info: Dict[str, Any] | None,
    ) -> tuple[OutPort | InPort, OutPort | InPort]:
        """Resolve concrete port objects for a connection, creating externals when needed."""

        def _existing_port(info: Dict[str, Any] | None, accessor) -> OutPort | InPort | None:
            if not info:
                return None
            port = info.get("port")
            if port is not None:
                return port
            instance = info.get("instance")
            if instance is None:
                return None
            return accessor(instance.link_manager, info["port_name"])

        from_port = _existing_port(from_info, lambda lm, name: lm.get_out_port(name))
        to_port = _existing_port(to_info, lambda lm, name: lm.get_in_port(name))

        if connection.type == ConnectionType.EXTERNAL_TO_INTERNAL:
            port_name = from_info.get("port_name") if from_info else connection.from_port_name
            from_port = InPort(port_name, to_port.msg_type, self.instance.namespace)
        elif connection.type == ConnectionType.INTERNAL_TO_EXTERNAL:
            port_name = to_info.get("port_name") if to_info else connection.to_port_name
            to_port = OutPort(port_name, from_port.msg_type, self.instance.namespace)
        else:
            # do nothing, both ports must exist
            pass
            
        if from_info is not None:
            from_info["port"] = from_port
        if to_info is not None:
            to_info["port"] = to_port

        return from_port, to_port

    def _create_wildcard_links(
        self,
        connection: Connection,
        port_list_from: Dict[str, Dict[str, Any]],
        port_list_to: Dict[str, Dict[str, Any]],
    ):
        """Create links for wildcard connections.
        
        Args:
            connection: Connection configuration
            port_list_from: Mapping of source port keys to metadata dictionaries
            port_list_to: Mapping of target port keys to metadata dictionaries
        """
        # filter the port name lists only for target instance
        from_idx = f"{connection.from_instance}.{connection.from_port_name}"
        to_idx = f"{connection.to_instance}.{connection.to_port_name}"

        # Match and pair ports based on wildcard patterns
        port_pairs = match_and_pair_wildcard_ports(
            from_idx,
            to_idx,
            port_list_from,
            port_list_to,
        )
        
        # Validate matched ports
        if not port_pairs:
            raise ValueError(
                f"No matching port pairs found: '{connection.from_port_name}' -> '{connection.to_port_name}'"
            )

        # Create links for each matched pair
        for from_key, to_key in port_pairs:
            from_info = port_list_from.get(from_key)
            to_info = port_list_to.get(to_key)

            if from_info is None or to_info is None:
                raise ValueError(f"Port metadata missing for pair: {from_key} -> {to_key}")

            from_port, to_port = self._resolve_ports_for_connection(connection, from_info, to_info)

            self._create_link_from_ports(from_port, to_port, connection.type)


    def set_links(self):
        """Set up links based on element connections."""
        connection_list: List[Connection] = []
        for cfg_connection in self.instance.configuration.connections:
            connection_instance = Connection(cfg_connection)
            connection_list.append(connection_instance)

        # dictionary of ports, having field of instance, port-name, port-type
        port_list_from: Dict[str, Dict[str, Any]] = {}
        port_list_to: Dict[str, Dict[str, Any]] = {}

        # ports from children instances
        for child_instance in self.instance.children.values():
            for port_name, port in child_instance.link_manager.in_ports.items():
                idx = f"{child_instance.name}.{port_name}"
                port_list_to[idx] = {"instance": child_instance, "port_name": port_name, "port": port}
            for port_name, port in child_instance.link_manager.out_ports.items():
                idx = f"{child_instance.name}.{port_name}"
                port_list_from[idx] = {"instance": child_instance, "port_name": port_name, "port": port}
        # ports from external interfaces
        external_interfaces = getattr(self.instance.configuration, "external_interfaces", None)
        if isinstance(external_interfaces, dict):
            for ext_input in external_interfaces.get("input", []):
                port_name = ext_input.get("name")
                idx = f".{port_name}"
                port_list_from[idx] = {"instance": None, "port_name": port_name, "port": None}
            for ext_output in external_interfaces.get("output", []):
                port_name = ext_output.get("name")
                idx = f".{port_name}"
                port_list_to[idx] = {"instance": None, "port_name": port_name, "port": None}

        # Establish links based on connection type
        for connection in connection_list:
            wildcard_fields = [connection.from_port_name, connection.to_port_name]
            if connection.from_instance:
                wildcard_fields.append(connection.from_instance)
            if connection.to_instance:
                wildcard_fields.append(connection.to_instance)

            has_wildcard = any("*" in field for field in wildcard_fields)

            if has_wildcard:
                self._create_wildcard_links(connection, port_list_from, port_list_to)
            else:
                # set from_port and to_port
                from_info = port_list_from.get(f"{connection.from_instance}.{connection.from_port_name}")
                to_info = port_list_to.get(f"{connection.to_instance}.{connection.to_port_name}")

                from_port, to_port = self._resolve_ports_for_connection(connection, from_info, to_info)
                self._create_link_from_ports(from_port, to_port, connection.type)

        # Create external ports after links are set
        self._create_external_ports()

    def _create_external_ports(self):
        """Create external ports based on link list."""
        # create in ports based on the link_list
        for link in self.links:
            # create port only if the namespace is the same as the instance
            # corresponds to external port
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
