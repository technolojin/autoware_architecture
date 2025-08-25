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

"""Port and connection models (placeholder for future implementation)."""

from typing import List, Dict, Any
import logging

logger = logging.getLogger(__name__)


class Port:
    """Base class for input/output ports (placeholder)."""
    
    def __init__(self, name: str, msg_type: str, namespace: List[str] = None):
        self.name = name
        self.msg_type = msg_type
        self.namespace = namespace or []
        self.topic = ""
    
    def __str__(self) -> str:
        return f"Port(name='{self.name}', type='{self.msg_type}')"


class InPort(Port):
    """Input port (placeholder)."""
    
    def __init__(self, name: str, msg_type: str, namespace: List[str] = None):
        super().__init__(name, msg_type, namespace)


class OutPort(Port):
    """Output port (placeholder)."""
    
    def __init__(self, name: str, msg_type: str, namespace: List[str] = None):
        super().__init__(name, msg_type, namespace)


class Link:
    """Link between ports (placeholder)."""
    
    def __init__(self, from_port: Port, to_port: Port, msg_type: str):
        self.from_port = from_port
        self.to_port = to_port
        self.msg_type = msg_type
    
    def __str__(self) -> str:
        return f"Link({self.from_port.name} -> {self.to_port.name})"


class Connection:
    """Connection configuration (placeholder)."""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.from_element = config.get("from", "")
        self.to_element = config.get("to", "")
    
    def __str__(self) -> str:
        return f"Connection({self.from_element} -> {self.to_element})"
