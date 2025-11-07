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


import os
import logging
from typing import Dict
from .config import ArchitectureConfig
from .models.config import Config
from .builder.config_registry import ConfigRegistry
from .builder.instances import DeploymentInstance
from .builder.launcher_generator import generate_pipeline_launch_file
from .builder.parameter_template_generator import ParameterTemplateGenerator
from .parsers.data_validator import element_name_decode
from .parsers.yaml_parser import yaml_parser
from .exceptions import ValidationError
from .template_utils import TemplateRenderer

logger = logging.getLogger(__name__)
debug_mode = True

class Deployment:
    def __init__(self, architecture_config: ArchitectureConfig ):
        # element collection
        architecture_yaml_list = self._get_architecture_list(architecture_config)
        self.config_registry = ConfigRegistry(architecture_yaml_list)

        # detect mode of input file (deployment vs architecture only)
        # if deployment_file ends with .architecture, it's an architecture-only file
        logger.info("deployment init Deployment file: %s", architecture_config.deployment_file)
        if architecture_config.deployment_file.endswith(".architecture"):
            logger.info("Detected architecture-only deployment file.")
            # need to parse the absolute path of the file from the config_registry
            # generate deployment config in-memory
            self.config_yaml_dir = architecture_config.deployment_file
            self.config_yaml = {}
            self.config_yaml['architecture'] = architecture_config.deployment_file
            self.config_yaml['name'] = architecture_config.deployment_file
            self.config_yaml.setdefault('vehicle_parameters', [])
            self.config_yaml.setdefault('environment_parameters', [])
            self.name = self.config_yaml.get("name")

        else:
            # input is a deployment file
            self.config_yaml_dir = architecture_config.deployment_file
            self.config_yaml = yaml_parser.load_config(self.config_yaml_dir)
            self.name = self.config_yaml.get("name")

        # Check the configuration
        self._check_config()

        # member variables - now supports multiple instances (one per mode)
        self.deploy_instances: Dict[str, DeploymentInstance] = {}  # mode_name -> DeploymentInstance
        self.vehicle_parameters_yaml = None
        self.sensor_calibration_yaml = None
        self.map_yaml = None

        # output paths
        self.output_root_dir = architecture_config.output_root_dir
        self.launcher_dir = os.path.join(self.output_root_dir, "exports", self.name, "launcher/")
        self.system_monitor_dir = os.path.join(self.output_root_dir, "exports", self.name, "system_monitor/")
        self.visualization_dir = os.path.join(self.output_root_dir, "exports", self.name,"visualization/")
        self.parameter_set_dir = os.path.join(self.output_root_dir, "exports", self.name,"parameter_set/")

        # build the deployment
        self.build()

        # set the vehicle individual parameters
        #   sensor calibration, vehicle parameters, map, etc.


    def _get_architecture_list(self, architecture_config: ArchitectureConfig) -> list[str]:
        architecture_list: list[str] = []
        manifest_dir = architecture_config.architecture_manifest_dir
        if not os.path.isdir(manifest_dir):
            raise ValidationError(f"Architecture manifest directory not found or not a directory: {manifest_dir}")

        # domains to include (always includes 'shared')
        domains_filter = set(architecture_config.effective_domains())
        logger.info(f"Domain filter active: {sorted(domains_filter)}")

        for entry in sorted(os.listdir(manifest_dir)):
            if not entry.endswith('.yaml'):
                continue
            manifest_file = os.path.join(manifest_dir, entry)
            try:
                manifest_yaml = yaml_parser.load_config(manifest_file)
                manifest_domain = manifest_yaml.get('domain', 'shared') # default to 'shared' if missing
                if manifest_domain not in domains_filter:
                    logger.debug(f"Skipping manifest '{entry}' (domain='{manifest_domain}' not in filter)")
                    continue
                files = manifest_yaml.get('architecture_config_files')
                # Allow the field to be empty or null without raising an error
                if files in (None, []):
                    logger.debug(
                        f"Manifest '{entry}' has empty architecture_config_files; skipping."
                    )
                    continue
                if not isinstance(files, list):
                    logger.warning(
                        f"Manifest '{entry}' has unexpected type for architecture_config_files: {type(files)}; skipping."
                    )
                    continue
                for f in files:
                    file_path = f.get('path') if isinstance(f, dict) else None
                    if file_path and file_path not in architecture_list:
                        architecture_list.append(file_path)
            except Exception as e:
                logger.warning(f"Failed to load manifest {manifest_file}: {e}")
        if not architecture_list:
            raise ValidationError(f"No architecture configuration files collected (domains={sorted(domains_filter)}).")
        return architecture_list

    def _check_config(self) -> bool:
        """Validate & normalize deployment configuration.

        Two supported input forms:
        1. Deployment YAML (fields: name, architecture, vehicle_parameters, environment_parameters)
        2. Raw Architecture YAML (only 'name' ending with '.architecture'). We synthesize a minimal
           deployment in-memory (no vehicles / environment parameters) so downstream logic works.
        """
        # Validate required fields now present
        for field in ['name', 'architecture']:
            if field not in self.config_yaml:
                raise ValidationError(
                    f"Field '{field}' is required in deployment configuration file {self.config_yaml_dir}"
                )

        # Optional lists: default to empty if omitted
        if 'vehicle_parameters' not in self.config_yaml:
            self.config_yaml['vehicle_parameters'] = []
        if 'environment_parameters' not in self.config_yaml:
            self.config_yaml['environment_parameters'] = []

        return True

    def build(self):
        # 1. Get architecture configuration
        architecture_name, _ = element_name_decode(self.config_yaml.get("architecture"))
        architecture = self.config_registry.get_architecture(architecture_name)

        if not architecture:
            raise ValidationError(f"Architecture not found: {architecture_name}")

        # 2. Determine modes to build
        modes_config = architecture.modes or []
        if modes_config:
            # Build one instance per mode
            mode_names = [m.get('name') for m in modes_config]
            logger.info(f"Building deployment for {len(mode_names)} modes: {mode_names}")
        else:
            # No modes defined - build single instance without mode filtering
            mode_names = [None]
            logger.info(f"Building deployment without mode filtering")

        # 3. Create deployment instance for each mode
        for mode_name in mode_names:
            try:
                mode_suffix = f"_{mode_name}" if mode_name else ""
                instance_name = f"{self.name}{mode_suffix}"
                deploy_instance = DeploymentInstance(instance_name, mode=mode_name)
                
                # Set architecture with mode filtering
                deploy_instance.set_architecture(
                    architecture, self.config_registry, mode=mode_name
                )
                
                # Store instance
                mode_key = mode_name if mode_name else "default"
                self.deploy_instances[mode_key] = deploy_instance
                logger.info(f"Successfully built deployment instance for mode: {mode_key}")
                
            except Exception as e:
                # try to visualize the architecture to show error status
                self.visualize()
                raise ValidationError(f"Error in setting deploy for mode '{mode_name}': {e}")

    def generate_by_template(self, data, template_path, output_dir, output_filename):
        """Generate file from template using the unified template renderer."""
        # Initialize template renderer
        renderer = TemplateRenderer()
        
        # Get template name from path
        template_name = os.path.basename(template_path)
        
        # Render template and save to file
        output_path = os.path.join(output_dir, output_filename)
        renderer.render_template_to_file(template_name, output_path, **data)

    def visualize(self):
        # 4. visualize the deployment diagram via plantuml
        # load the template file
        template_dir = os.path.join(os.path.dirname(__file__), "../template")
        node_template_path = os.path.join(template_dir, "node_diagram.puml.jinja2")
        logic_template_path = os.path.join(template_dir, "logic_diagram.puml.jinja2")
        sequence_template_path = os.path.join(template_dir, "sequence_diagram.puml.jinja2")

        # Generate visualization for each mode
        for mode_key, deploy_instance in self.deploy_instances.items():
            # Collect data from the architecture instance
            data = deploy_instance.collect_instance_data()
            
            # Create mode-specific output directory
            mode_visualization_dir = os.path.join(self.visualization_dir, mode_key)
            
            # Generate diagrams with mode suffix in filename
            filename_base = f"{self.name}_{mode_key}" if mode_key != "default" else self.name
            self.generate_by_template(data, node_template_path, mode_visualization_dir, filename_base + "_node_graph.puml")
            self.generate_by_template(data, logic_template_path, mode_visualization_dir, filename_base + "_logic_graph.puml")
            self.generate_by_template(data, sequence_template_path, mode_visualization_dir, filename_base + "_sequence_graph.puml")
            
            logger.info(f"Generated visualization for mode: {mode_key}")

    def generate_system_monitor(self):
        # load the template file
        template_dir = os.path.join(os.path.dirname(__file__), "../template")
        topics_template_path = os.path.join(template_dir, "sys_monitor_topics.yaml.jinja2")

        # Generate system monitor for each mode
        for mode_key, deploy_instance in self.deploy_instances.items():
            # Collect data from the architecture instance
            data = deploy_instance.collect_instance_data()
            
            # Create mode-specific output directory
            mode_monitor_dir = os.path.join(self.system_monitor_dir, mode_key, "component_state_monitor")
            self.generate_by_template(data, topics_template_path, mode_monitor_dir, "topics.yaml")
            
            logger.info(f"Generated system monitor for mode: {mode_key}")


    def generate_launcher(self):
        # Generate launcher files for each mode
        for mode_key, deploy_instance in self.deploy_instances.items():
            # Create mode-specific launcher directory
            mode_launcher_dir = os.path.join(self.launcher_dir, mode_key)
            
            # Generate pipeline launch files
            generate_pipeline_launch_file(deploy_instance, mode_launcher_dir)
            
            logger.info(f"Generated launcher for mode: {mode_key}")

    def generate_parameter_set_template(self):
        """Generate parameter set template using ParameterTemplateGenerator."""
        if not self.deploy_instances:
            raise ValidationError("Deployment instances are not initialized")
        
        # Generate parameter set template for each mode
        output_paths = {}
        for mode_key, deploy_instance in self.deploy_instances.items():
            # Create mode-specific output directory
            mode_parameter_dir = os.path.join(self.parameter_set_dir, mode_key)
            os.makedirs(mode_parameter_dir, exist_ok=True)
            
            # Initialize template renderer
            renderer = TemplateRenderer()
            
            # Create parameter template generator and generate the template
            generator = ParameterTemplateGenerator(deploy_instance)
            template_name = f"{self.name}_{mode_key}" if mode_key != "default" else self.name
            output_path_list = generator.generate_parameter_set_template(
                template_name,
                renderer,
                mode_parameter_dir
            )

            output_paths[mode_key] = output_path_list
            logger.info(f"Generated {len(output_path_list)} parameter set templates for mode: {mode_key}")
        
        return output_paths
