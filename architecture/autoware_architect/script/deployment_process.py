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
from typing import List

from autoware_architect.deployment import Deployment
from autoware_architect.config import ArchitectureConfig


def _parse_domains(domains_arg: str) -> List[str]:
    """Parse a domain argument which may be:
    - single token (e.g. "example")
    - bracket list: "[example, deploy_a]"
    - comma or semicolon separated list: "example,deploy_a" or "example;deploy_a"
    Always append 'shared' if missing.
    """
    if not domains_arg:
        return ["shared"]
    s = domains_arg.strip()
    if s.startswith("[") and s.endswith("]"):
        s = s[1:-1]
    # unify separators to comma
    s = s.replace(";", ",")
    parts = [p.strip() for p in s.split(",") if p.strip()]
    if "shared" not in parts:
        parts.append("shared")
    return parts

# build the deployment
# search and connect the connections between the modules
def build(deployment_file: str, architecture_manifest_dir: str, output_root_dir: str, domains_raw: str):
    # Inputs:
    #   deployment_file: YAML deployment configuration
    #   architecture_manifest_dir: directory containing per-package manifest YAML files (each lists architecture_config_files)
    #   output_root_dir: root directory for generated exports

    # configure the architecture
    architecture_config = ArchitectureConfig()
    architecture_config.debug_mode = True
    architecture_config.log_level = "INFO"

    architecture_config.deployment_file = deployment_file
    architecture_config.architecture_manifest_dir = architecture_manifest_dir
    architecture_config.output_root_dir = output_root_dir
    parsed_domains = _parse_domains(domains_raw)
    architecture_config.domains = parsed_domains

    logger = architecture_config.set_logging()

    # load and build the deployment
    logger.info("autoware architect: Building deployment...")
    deployment = Deployment(architecture_config)

    # parameter set template export
    logger.info("autoware architect: Exporting parameter set template...")
    deployment.generate_parameter_set_template()

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
    # Accept variable number of domain arguments:
    #   deployment_process.py <deployment_file> <manifest_dir> <output_root_dir> <domain1> [domain2 ...]
    # If multiple domains passed as separate args, we aggregate them.
    if len(sys.argv) < 5:
        raise SystemExit("Usage: deployment_process.py <deployment_file> <manifest_dir> <output_root_dir> <domain1> [domain2 ...]")
    if len(sys.argv) == 5:
        domains_arg = sys.argv[4]
    else:
        # Join remaining domains into bracket list for parser clarity
        domains_arg = "[" + ",".join(sys.argv[4:]) + "]"
    build(sys.argv[1], sys.argv[2], sys.argv[3], domains_arg)
