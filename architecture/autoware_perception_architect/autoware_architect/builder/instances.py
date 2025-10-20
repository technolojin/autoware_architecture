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

from ..models.config import Config, ModuleConfig, PipelineConfig, ParameterSetConfig, ArchitectureConfig
from ..parsers.data_parser import element_name_decode
from ..config import config
from ..exceptions import ValidationError
from ..utils.naming import generate_unique_id
from .config_registry import ConfigRegistry
from .parameter_manager import ParameterManager
from .link_manager import LinkManager
from .event_manager import EventManager

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
        self.namespace_str: str = "/" + "/".join(self.namespace)

        self.compute_unit: str = compute_unit
        self.layer: int = layer
        if self.layer > config.layer_limit:
            raise ValidationError(f"Instance layer is too deep (limit: {config.layer_limit})")

        # configuration
        self.configuration: ModuleConfig | PipelineConfig | ParameterSetConfig | ArchitectureConfig | None = None

        # instance topology
        self.element_type: str = None
        self.parent: Instance = None
        self.children: Dict[str, Instance] = {}
        self.parent_pipeline_list: List[str] = []

        # interface
        self.link_manager: LinkManager = LinkManager(self)

        # parameter manager
        self.parameter_manager: ParameterManager = ParameterManager(self)

        # event manager
        self.event_manager: EventManager = EventManager(self)

        # status
        self.is_initialized = False

    @property
    def unique_id(self):
        return generate_unique_id(self.namespace, self.name)

    def set_instances(self, element_id: str, config_registry: ConfigRegistry):

        try:
            element_name, element_type = element_name_decode(element_id)
            if element_type == "architecture":
                self._set_architecture_instances(config_registry)
            elif element_type == "pipeline":
                self._set_pipeline_instances(element_id, element_name, config_registry)
            elif element_type == "module":
                self._set_module_instances(element_id, element_name, config_registry)
        except Exception as e:
            raise ValidationError(f"Error setting instances for {element_id}, at {self.configuration.file_path}")

    def _set_architecture_instances(self, config_registry: ConfigRegistry):
        """Set instances for architecture element type."""
        # First pass: create all component instances
        for cfg_component in self.configuration.components:
            compute_unit_name = cfg_component.get("compute_unit")
            instance_name = cfg_component.get("component")
            element_id = cfg_component.get("element")
            namespace = cfg_component.get("namespace")
            if namespace:
                if isinstance(namespace, str):
                    namespace = namespace.split('/') if '/' in namespace else [namespace]
            else:
                namespace = []

            # create instance
            instance = Instance(instance_name, compute_unit_name, namespace)
            instance.parent = self
            try:
                instance.set_instances(element_id, config_registry)
            except Exception as e:
                # add the instance to the children dict for debugging
                self.children[instance_name] = instance
                raise ValidationError(f"Error in setting component instance '{instance_name}', at {self.configuration.file_path}")

            self.children[instance_name] = instance
            logger.debug(f"Architecture instance '{self.namespace_str}' added component '{instance_name}' (uid={instance.unique_id})")
        
        # Second pass: apply parameter sets after all instances are created
        # This ensures that parameter_sets can target nodes across different components
        for cfg_component in self.configuration.components:
            instance_name = cfg_component.get("component")
            instance = self.children[instance_name]
            self._apply_parameter_set(instance, cfg_component, config_registry)
        
        # all children are initialized
        self.is_initialized = True

    def _set_pipeline_instances(self, element_id: str, element_name: str, config_registry: ConfigRegistry):
        """Set instances for pipeline element type."""
        logger.info(f"Setting pipeline element {element_id} for instance {self.namespace_str}")
        self.configuration = config_registry.get_pipeline(element_name)
        self.element_type = "pipeline"

        # check if the pipeline is already set
        if element_id in self.parent_pipeline_list:
            raise ValidationError(f"Config is already set: {element_id}, avoid circular reference")
        self.parent_pipeline_list.append(element_id)

        # set children
        self._create_pipeline_children(config_registry)

        # run the pipeline configuration
        self._run_pipeline_configuration()

        # recursive call is finished
        self.is_initialized = True

    def _set_module_instances(self, element_id: str, element_name: str, config_registry: ConfigRegistry):
        """Set instances for module element type."""
        logger.info(f"Setting module element {element_id} for instance {self.namespace_str}")
        self.configuration = config_registry.get_module(element_name)
        self.element_type = "module"

        # run the module configuration
        self._run_module_configuration()

        # recursive call is finished
        self.is_initialized = True

    def _apply_parameter_set(self, instance: "Instance", cfg_component: dict, config_registry: ConfigRegistry):
        """Apply parameter set to an instance using direct node targeting.
        
        Only applies parameters to nodes that are descendants of the given instance.
        """
        parameter_set = cfg_component.get("parameter_set")
        cfg_param_set: ParameterSetConfig = None
        if parameter_set is not None:
            param_set_name, element_type = element_name_decode(parameter_set)
            if element_type != "parameter_set":
                raise ValidationError(f"Invalid parameter set type: {element_type}, at {self.configuration.file_path}")
            cfg_param_set = config_registry.get_parameter_set(param_set_name)
        # apply the parameter set to the instance        
        try:
            if cfg_param_set is not None:
                node_params = cfg_param_set.parameters
                logger.info(f"Applying parameter set '{param_set_name}' to component '{instance.name}'")
                
                # New parameter set format: direct node targeting
                for param_config in node_params:
                    if isinstance(param_config, dict) and "node" in param_config:
                        # New format: direct node targeting with absolute namespace
                        node_namespace = param_config.get("node")
                        
                        # Only apply if the target node is under this component's namespace
                        if not node_namespace.startswith(instance.namespace_str + "/"):
                            logger.debug(f"Parameter set '{param_set_name}' skip node '{node_namespace}' (component namespace '{instance.namespace_str}')")
                            continue
                        
                        parameter_files_raw = param_config.get("parameter_files", [])
                        configurations = param_config.get("configurations", [])
                        
                        # Validate parameter_files format (should be list of dicts)
                        parameter_files = []
                        if parameter_files_raw:
                            for pf in parameter_files_raw:
                                if isinstance(pf, dict):
                                    parameter_files.append(pf)
                                else:
                                    logger.warning(f"Invalid parameter_files format in parameter set '{param_set_name}': {pf}")
                        
                        # Apply parameters directly to the target node
                        instance.parameter_manager.apply_node_parameters(
                            node_namespace, parameter_files, configurations
                        )
                        logger.debug(f"Applied parameters to node '{node_namespace}' from set '{param_set_name}' files={len(parameter_files)} configs={len(configurations)}")
        except Exception as e:
            raise ValidationError(f"Error in applying parameter set '{param_set_name}' to instance '{instance.name}': {e}")

    def _create_pipeline_children(self, config_registry: ConfigRegistry):
        """Create child instances for pipeline elements."""
        cfg_node_list = self.configuration.nodes
        for cfg_node in cfg_node_list:
            instance = Instance(
                cfg_node.get("node"), self.compute_unit, self.namespace, self.layer + 1
            )
            instance.parent = self
            instance.parent_pipeline_list = self.parent_pipeline_list.copy()
            # recursive call of set_instances
            try:
                instance.set_instances(cfg_node.get("element"), config_registry)
            except Exception as e:
                # add the instance to the children dict for debugging
                self.children[instance.name] = instance
                raise ValidationError(f"Error in setting child instance {instance.name} : {e}, at {self.configuration.file_path}")
            self.children[instance.name] = instance
        
    def _run_pipeline_configuration(self):
        if self.element_type != "pipeline":
            raise ValidationError(f"run_pipeline_configuration is only supported for pipeline, at {self.configuration.file_path}")

        # set connections
        if len(self.configuration.connections) == 0:
            raise ValidationError(f"No connections found in the pipeline configuration, at {self.configuration.file_path}")

        self.link_manager.set_links()

        # create external ports
        self.link_manager.create_external_ports(self.link_manager.links)

        # log pipeline configuration
        self.link_manager.log_pipeline_configuration()

    def _run_module_configuration(self):
        if self.element_type != "module":
            raise ValidationError(f"run_module_configuration is only supported for module, at {self.configuration.file_path}")

        # set ports
        self.link_manager.initialize_module_ports()

        # set parameters
        self.parameter_manager.initialize_module_parameters()

        # initialize processes and events
        self.event_manager.initialize_module_processes()

    def get_child(self, name: str):
        if name in self.children:
            return self.children[name]
        raise ValidationError(f"Child not found: child name '{name}', instance of '{self.name}'")

    def check_ports(self):
        # recursive call for children
        for child in self.children.values():
            child.check_ports()

        # delegate to link manager
        self.link_manager.check_ports()

    def set_event_tree(self):
        # delegate to event manager
        self.event_manager.set_event_tree()

    def collect_instance_data(self) -> dict:
        data = {
            "name": self.name,
            "unique_id": self.unique_id,
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
            "events": self.event_manager.get_all_events(),
            "parameters": self.parameter_manager.get_all_parameter_files(),
        }

        return data

class DeploymentInstance(Instance):
    def __init__(self, name: str):
        super().__init__(name)

    def set_architecture(
        self,
        architecture: Config,
        config_registry,
    ):
        logger.info(f"Setting architecture {architecture.full_name} for instance {self.name}")
        self.configuration = architecture
        self.element_type = "architecture"

        # 1. set component instances
        logger.info(f"Instance '{self.name}': setting component instances")
        self.set_instances(architecture.full_name, config_registry)

        # 2. set connections
        logger.info(f"Instance '{self.name}': setting connections")
        self.link_manager.set_links()
        self.check_ports()

        # 3. build logical topology
        logger.info(f"Instance '{self.name}': building logical topology")
        # self.build_logical_topology()
        self.set_event_tree()

