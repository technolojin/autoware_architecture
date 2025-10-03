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

from .config import ArchitectureConfig
# from .models import elements as awa_cls
from .models.elements import load_config_yaml
from .models.elements import ModuleList, ElementList, PipelineList, ParameterSetList, ArchitectureList
from .builder.instances import DeploymentInstance
import jinja2

debug_mode = True
class Deployment:
    def __init__(self, architecture_config: ArchitectureConfig ):

        # parse the architecture yaml configuration list
        # the list is a text file that contains directories of the yaml files
        with open(architecture_config.architecture_yaml_list_file, "r") as file:
            architecture_yaml_list = file.read().splitlines()

        # load yaml file
        self.config_yaml_dir = architecture_config.deployment_file
        self.config_yaml = load_config_yaml(self.config_yaml_dir)
        self.name = self.config_yaml.get("name")

        # element lists
        element_list = ElementList(architecture_yaml_list)

        self.module_list: ModuleList = ModuleList(element_list.get_module_list())
        self.pipeline_list: PipelineList = PipelineList(
            element_list.get_pipeline_list()
        )
        self.parameter_set_list: ParameterSetList = ParameterSetList(
            element_list.get_parameter_set_list()
        )
        self.architecture_list: ArchitectureList = ArchitectureList(
            element_list.get_architecture_list()
        )

        # Check the configuration
        self._check_config()

        # member variables
        self.deploy_instance: DeploymentInstance = None
        self.vehicle_parameters_yaml = None
        self.sensor_calibration_yaml = None
        self.map_yaml = None

        # output paths
        self.output_root_dir = architecture_config.output_root_dir
        self.launcher_dir = os.path.join(self.output_root_dir, "exports", self.name, "launcher/")
        self.system_monitor_dir = os.path.join(self.output_root_dir, "exports", self.name, "system_monitor/")
        self.visualization_dir = os.path.join(self.output_root_dir, "exports", self.name,"visualization/")

        # build the deployment
        self.build()

        # set the vehicle individual parameters
        #   sensor calibration, vehicle parameters, map, etc.

    def _check_config(self) -> bool:
        # Check the name field
        deployment_config_fields = [
            "name",
            "architecture",
            "vehicle_parameters",
            "environment_parameters",
        ]
        for field in deployment_config_fields:
            if field not in self.config_yaml:
                raise ValueError(
                    f"Field '{field}' is required in deployment configuration file {self.config_yaml_dir}"
                )
                return False
        return True

    def build(self):
        # 1. set architecture instance
        architecture = self.architecture_list.get(self.config_yaml.get("architecture"))

        if not architecture:
            raise ValueError(f"Architecture not found: {self.config_yaml.get('architecture')}")

        try:
            self.deploy_instance = DeploymentInstance(self.name)
            # 1. set deploy instance
            self.deploy_instance.set_architecture(
                architecture, self.module_list, self.pipeline_list, self.parameter_set_list
            )
            # 2. set connections
            self.deploy_instance.set_connections()
            # 3. build the logical topology
            self.deploy_instance.build_logical_topology()
        except Exception as e:
            # try to visualize the architecture to show error status
            self.visualize()
            raise ValueError(f"Error in setting deploy: {e}")

    def generate_by_template(self, data, template_path, output_dir, output_filename):
        # load the template file
        with open(template_path, "r") as f:
            template_file = f.read()

        # Render the Jinja2 template with the collected data
        template = jinja2.Template(template_file)
        output = template.render(data)

        # write the plantuml file
        output_path = os.path.join(output_dir, output_filename)
        if os.path.exists(output_path):
            os.remove(output_path)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)
        with open(output_path, "w") as f:
            f.write(output)

    def visualize(self):
        # 4. visualize the deployment diagram via plantuml
        # define the traverse_instance function

        # load the template file
        template_dir = os.path.join(os.path.dirname(__file__), "../template")
        node_template_path = os.path.join(template_dir, "node_diagram.puml.jinja2")
        logic_template_path = os.path.join(template_dir, "logic_diagram.puml.jinja2")
        sequence_template_path = os.path.join(template_dir, "sequence_diagram.puml.jinja2")

        # Collect data from the architecture instance
        data = self.deploy_instance.collect_instance_data()

        # draw node diagram
        self.generate_by_template(data, node_template_path, self.visualization_dir, self.name + "_node_graph.puml")
        self.generate_by_template(data, logic_template_path, self.visualization_dir, self.name + "_logic_graph.puml")
        self.generate_by_template(data, sequence_template_path, self.visualization_dir, self.name + "_sequence_graph.puml")

    def generate_system_monitor(self):
        # load the template file
        template_dir = os.path.join(os.path.dirname(__file__), "../template")
        topics_template_path = os.path.join(template_dir, "sys_monitor_topics.yaml.jinja2")

        # Collect data from the architecture instance
        data = self.deploy_instance.collect_instance_data()

        file_out_dir = os.path.join(self.system_monitor_dir, "component_state_monitor")
        self.generate_by_template(data, topics_template_path, file_out_dir, "topics.yaml")


    def generate_launcher(self):
        # 3. build the launcher
        pass
