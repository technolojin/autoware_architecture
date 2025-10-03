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

import sys

from autoware_architect.instance import Deployment
from autoware_architect.config import ArchitectureConfig

# build the deployment
# search and connect the connections between the modules
def build(deployment_file: str, architecture_yaml_list_file: str, output_root_dir: str):
    # Inputs:
    #   deployment_file: a yaml file that contains the deployment configuration
    #   architecture_yaml_list: a list of yaml file directories that contain the architecture configuration
    #   output_root_dir: the root directory for the output files

    # configure the architecture
    architecture_config = ArchitectureConfig()
    architecture_config.debug_mode = True
    architecture_config.log_level = "INFO"

    architecture_config.deployment_file = deployment_file
    architecture_config.architecture_yaml_list_file = architecture_yaml_list_file
    architecture_config.output_root_dir = output_root_dir

    logger = architecture_config.set_logging()

    # load and build the deployment
    logger.info("autoware architect: Building deployment...")
    deployment = Deployment(architecture_config)

    # generate the system visualization
    logger.info("autoware architect: Generating visualization...")
    deployment.visualize()

    # generate the launch files
    logger.info("autoware architect: Generating launch files...")
    deployment.generate_launcher()

    # generate the system monitor configuration
    logger.info("autoware architect: Generating system monitor configuration...")
    deployment.generate_system_monitor()

    logger.info("autoware architect: Done!")


if __name__ == "__main__":
    build(sys.argv[1], sys.argv[2], sys.argv[3])
