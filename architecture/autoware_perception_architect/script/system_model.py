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

from logging import getLogger
from typing import List

logger = getLogger("__name__")


class Event:
    def __init__(self, name: str, namespace: List[str], is_process_event=False):
        self.name = name
        self.namespace = namespace
        self.id = ("__".join(namespace) + "__" + name).replace("/", "__")
        self.type_list = [
            "on_input",
            "on_trigger",
            "once",
            "periodic",
            "to_trigger",
            "to_output",
        ]
        self.type: str = None
        self.process_event: bool = is_process_event
        self.triggers: List["Event"] = []
        self.actions: List["Event"] = []
        self.trigger_root_ids: List[str] = []
        self.frequency: float = None
        self.warn_rate: float = None
        self.error_rate: float = None
        self.timeout: float = None
        self.is_set: bool = False

    def set_type(self, type_str):
        if type_str not in self.type_list:
            raise ValueError(f"Invalid event type: {type_str}")
        self.type = type_str

    def check_trigger_root_ids(self, trigger_root_id):
        if trigger_root_id in self.trigger_root_ids:
            return True
        else:
            self.trigger_root_ids.append(trigger_root_id)
            return False

    def add_trigger_event(self, event, vise_versa=True):
        if event.id == self.id:
            raise ValueError(f"Event cannot trigger itself: {self.id}")
        if any(e.id == event.id for e in self.triggers):
            return
        self.triggers.append(event)
        if vise_versa:
            event.add_action_event(self, False)

    def add_action_event(self, event, vise_versa=True):
        if event.id == self.id:
            raise ValueError(f"Event cannot trigger itself: {self.id}")
        if any(e.id == event.id for e in self.actions):
            return
        self.actions.append(event)
        if vise_versa:
            event.add_trigger_event(self, False)

    def determine_type(self, config_yaml):
        if not config_yaml:
            raise ValueError("Config is empty")
        first_config = list(config_yaml)[0]
        if isinstance(first_config, str):
            return first_config, config_yaml.get(first_config)
        if isinstance(first_config, dict):
            type_key = list(first_config.keys())[0]
            return type_key, first_config[type_key]
        return None, None

    def set_trigger(
        self,
        config_yaml,
        process_list: List["Event"],
        on_input_list: List["Event"],
    ):
        config_key, config_value = self.determine_type(config_yaml)

        if isinstance(config_yaml, list) and len(config_yaml) == 1:
            config_yaml = config_yaml[0]

        if config_key in self.type_list:
            if config_key == "periodic":
                self.frequency = config_value
                self.is_set = True
            elif config_key == "once" and config_value is None:
                self.frequency = 0.0
                self.warn_rate = 0.0
                self.error_rate = 0.0
                self.timeout = 0.0
                self.is_set = True
            elif config_key in ["on_input", "once"]:
                self.condition_value = config_value
                if not any(
                    event.name == f"input_{config_value}" and self.add_trigger_event(event)
                    for event in on_input_list
                ):
                    raise ValueError(f"Input event not found: {config_value}")
            elif config_key == "on_trigger":
                self.condition_value = config_value
                if not any(
                    event.name == config_value and self.add_trigger_event(event)
                    for event in process_list
                ):
                    raise ValueError(f"Trigger event not found: {config_value}")
            else:
                raise ValueError(f"Invalid event type to set trigger: {config_key}")

            if "warn_rate" in config_yaml:
                self.warn_rate = config_yaml.get("warn_rate")
            if "error_rate" in config_yaml:
                self.error_rate = config_yaml.get("error_rate")
            if "timeout" in config_yaml:
                self.timeout = config_yaml.get("timeout")
        else:
            raise ValueError(f"Invalid event type: {config_key}")

    def set_event_frequency(
        self,
        trigger_root_id: str,
        frequency: float,
        warn_rate: float,
        error_rate: float,
        timeout: float,
    ):
        if self.check_trigger_root_ids(trigger_root_id):
            return

        if frequency is not None and (self.frequency is None or frequency > self.frequency):
            self.frequency = frequency
        if warn_rate is not None and (self.warn_rate is None or warn_rate > self.warn_rate):
            self.warn_rate = warn_rate
        if error_rate is not None and (self.error_rate is None or error_rate > self.error_rate):
            self.error_rate = error_rate
        if timeout is not None and (self.timeout is None or timeout > self.timeout):
            self.timeout = timeout

        self.is_set = True

        for action in self.actions:
            action.set_event_frequency(trigger_root_id, frequency, warn_rate, error_rate, timeout)

    def set_frequency_tree(self):
        trigger_root_id = self.id
        if self.type == "periodic" and self.is_set:
            for action in self.actions:
                action.set_event_frequency(
                    trigger_root_id, self.frequency, self.warn_rate, self.error_rate, self.timeout
                )
        elif self.type == "once" and self.is_set:
            for action in self.actions:
                action.set_event_frequency(trigger_root_id, 0.0, 0.0, 0.0, 0.0)


class EventChain(Event):
    def __init__(self, name: str, namespace: List[str] = [], is_process_event=True):
        super().__init__(name, namespace, is_process_event)
        self.chain_list = ["and", "or"]
        self.children: List[Event] = []

    def set_type(self, type_str):
        if type_str not in self.chain_list + self.type_list:
            raise ValueError(f"Invalid event chain type: {type_str}")
        self.type = type_str

    def set_chain(
        self,
        config_yaml,
        process_list: List[Event],
        on_input_list: List[Event],
        child_idx=0,
    ):
        if isinstance(config_yaml, dict):
            config_key, config_value = self.determine_type(config_yaml)
            if config_key in self.type_list:
                self.set_type(config_key)
                self.set_trigger(config_yaml, process_list, on_input_list)
            elif config_key in self.chain_list:
                if len(config_value) == 1:
                    self.set_chain(config_value[0], process_list, on_input_list)
                else:
                    self.set_type(config_key)
                    for chain in config_value:
                        chain_key, _ = self.determine_type(chain)
                        if chain_key in ["periodic"] + self.chain_list:
                            event = EventChain(f"{self.name}_{child_idx}", self.namespace)
                            child_idx += 1
                            event.set_chain(chain, process_list, on_input_list)
                            self.add_trigger_event(event)
                            self.children.append(event)
                        elif chain_key in self.type_list:
                            self.set_trigger(chain, process_list, on_input_list)
                        else:
                            raise ValueError(f"Invalid trigger condition type: {chain_key}")
        elif isinstance(config_yaml, list):
            if len(config_yaml) == 1:
                self.set_chain(config_yaml[0], process_list, on_input_list)
            else:
                self.set_chain({"or": config_yaml}, process_list, on_input_list)

    def get_children(self):
        children_list = []
        for child in self.children:
            children_list.extend(child.get_children())
        return children_list + self.children


class Process:
    def __init__(self, name: str, namespace: List[str], config_yaml: dict):
        self.name = name
        self.namespace = namespace
        self.config_yaml = config_yaml
        self.event: EventChain = EventChain(name, namespace)
        self.id = f'{"__".join(namespace)}__process__{name}'.replace("/", "__")

    def set_condition(self, process_list, on_input_list):
        trigger_condition_config = self.config_yaml.get("trigger_conditions")
        self.event.set_chain(trigger_condition_config, process_list, on_input_list)

    def set_outcomes(self, process_list, to_output_events):
        for outcome in self.config_yaml.get("outcomes"):
            outcome_type, outcome_value = list(outcome.items())[0]
            if outcome_type == "to_output":
                if not any(
                    event.name == f"output_{outcome_value}" and self.event.add_action_event(event)
                    for event in to_output_events
                ):
                    raise ValueError(f"Output event not found: {outcome_value}")
            elif outcome_type == "to_trigger":
                if not any(
                    event.name == outcome_value and self.event.add_action_event(event)
                    for event in process_list
                ):
                    raise ValueError(f"Trigger event not found: {outcome_value}")

    def get_event_list(self):
        return self.event.get_children() + [self.event]


class Port:
    def __init__(self, name: str, msg_type: str, namespace: List[str] = []):
        self.name = name
        self.msg_type = msg_type
        self.namespace = namespace
        self.full_name = f'/{"/".join(namespace)}/{name}'
        self.reference: List["Port"] = []
        self.topic: List[str] = []
        self.event = None
        self.is_global = False

    def set_references(self, port_list: List["Port"]):
        for port in port_list:
            if port.full_name not in [p.full_name for p in self.reference]:
                self.reference.append(port)

    def get_reference_list(self):
        return self.reference or [self]

    def set_topic(self, topic_namespace: List[str], topic_name: str):
        self.topic = topic_namespace + [topic_name]


class InPort(Port):
    def __init__(self, name, msg_type, namespace: List[str] = []):
        super().__init__(name, msg_type, namespace)
        self.full_name = f'/{"/".join(namespace)}/input/{name}'
        self.id = f'{"__".join(namespace)}__input_{name}'.replace("/", "__")
        self.is_required = True
        self.servers: List[Port] = []
        self.event = Event(f"input_{name}", namespace)
        self.event.set_type("on_input")

    def set_servers(self, port_list: List[Port]):
        for port in port_list:
            if port.full_name not in [p.full_name for p in self.servers]:
                self.servers.append(port)


class OutPort(Port):
    def __init__(self, name, msg_type, namespace: List[str] = []):
        super().__init__(name, msg_type, namespace)
        self.full_name = f'/{"/".join(namespace)}/output/{name}'
        self.id = f'{"__".join(namespace)}__output_{name}'.replace("/", "__")
        self.frequency = 0.0
        self.is_monitored = False
        self.users: List[Port] = []
        self.event = Event(f"output_{name}", namespace)
        self.event.set_type("to_output")

    def set_users(self, port_list: List[Port]):
        for port in port_list:
            if port.full_name not in [p.full_name for p in self.users]:
                self.users.append(port)


class Link:
    def __init__(self, msg_type: str, from_port: Port, to_port: Port, namespace: List[str] = []):
        self.msg_type = msg_type
        self.from_port = from_port
        self.to_port = to_port
        self.namespace = namespace
        self.check_connection()

    def check_connection(self):
        is_from_internal = isinstance(self.from_port, OutPort)
        is_to_internal = isinstance(self.to_port, InPort)

        if is_from_internal and is_to_internal:
            from_ports = self.from_port.get_reference_list()
            to_ports = self.to_port.get_reference_list()

            for port in from_ports + to_ports:
                if port.msg_type != self.msg_type:
                    raise ValueError(f"Type mismatch for {port.name}")

            for from_ref in from_ports:
                from_ref.set_users(to_ports)
                from_ref.set_topic(self.from_port.namespace, self.from_port.name)
            for to_ref in to_ports:
                to_ref.set_servers(from_ports)
                to_ref.set_topic(self.from_port.namespace, self.from_port.name)
                for server_port in to_ref.servers:
                    to_ref.event.add_trigger_event(server_port.event)

        elif is_from_internal and not is_to_internal:
            from_refs = self.from_port.get_reference_list()
            self.to_port.set_references(from_refs)
            for ref in from_refs:
                ref.set_topic(self.to_port.namespace, self.to_port.name)

        elif not is_from_internal and is_to_internal:
            to_refs = self.to_port.get_reference_list()
            self.from_port.set_references(to_refs)
        else:
            raise ValueError(f"Invalid connection: {self.from_port.name} -> {self.to_port.name}")


class Connection:
    def __init__(self, connection_dict: dict):
        self.from_instance, self.from_port_name = self.parse_port_name(
            connection_dict.get("from")
        )
        self.to_instance, self.to_port_name = self.parse_port_name(connection_dict.get("to"))

        if not self.from_instance and not self.to_instance:
            raise ValueError(f"Invalid connection: {connection_dict}")
        self.type = (2 if self.from_instance and self.to_instance else (3 if self.from_instance else 1))

    def parse_port_name(self, port_name: str) -> (str, str):
        parts = port_name.split(".")
        if len(parts) == 2 and parts[0] in ["input", "output"]:
            return "", parts[1]
        if len(parts) == 3 and parts[1] in ["input", "output"]:
            return parts[0], parts[2]
        raise ValueError(f"Invalid port name: {port_name}")


class Parameter:
    def __init__(self, name: str, value: str):
        self.name = name
        self.value = value


class ParameterList:
    def __init__(self):
        self.list: List[Parameter] = []

    def get_parameter(self, parameter_name):
        for p in self.list:
            if p.name == parameter_name:
                return p.value
        return None

    def set_parameter(self, parameter_name, parameter_value):
        for p in self.list:
            if p.name == parameter_name:
                p.value = parameter_value
                return
        self.list.append(Parameter(parameter_name, parameter_value))
