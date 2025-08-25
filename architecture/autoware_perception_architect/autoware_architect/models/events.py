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

"""Event models (placeholder for future implementation)."""

from typing import List, Dict, Any
import logging

logger = logging.getLogger(__name__)


class Event:
    """Event model (placeholder)."""
    
    def __init__(self, name: str, namespace: List[str] = None, is_process_event: bool = False):
        self.name = name
        self.namespace = namespace or []
        self.is_process_event = is_process_event
        self.type = None
        self.frequency = None
    
    def __str__(self) -> str:
        return f"Event(name='{self.name}', type='{self.type}')"


class EventChain(Event):
    """Event chain model (placeholder)."""
    
    def __init__(self, name: str, namespace: List[str] = None):
        super().__init__(name, namespace, is_process_event=True)
        self.children = []
    
    def __str__(self) -> str:
        return f"EventChain(name='{self.name}', children={len(self.children)})"


class Process:
    """Process model (placeholder)."""
    
    def __init__(self, name: str, namespace: List[str] = None, config: Dict[str, Any] = None):
        self.name = name
        self.namespace = namespace or []
        self.config = config or {}
        self.events = []
    
    def __str__(self) -> str:
        return f"Process(name='{self.name}', events={len(self.events)})"
