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
from ..builder.instances import DeploymentInstance
from ..template_utils import TemplateRenderer

logger = logging.getLogger(__name__)


def visualize_deployment(deploy_instances: Dict[str, DeploymentInstance], name: str, visualization_dir: str):
    """Generate visualization files for deployment instances.

    Args:
        deploy_instances: Dictionary mapping mode names to deployment instances
        name: Base name for the deployment
        visualization_dir: Directory to output visualization files
    """
    # load the template file
    template_dir = os.path.join(os.path.dirname(__file__), "../template")
    node_dot_template_path = os.path.join(template_dir, "node_diagram.dot.jinja2")
    logit_dot_template_path = os.path.join(template_dir, "logic_diagram.dot.jinja2")

    # Web visualization templates
    web_data_template_path = os.path.join(template_dir, "visualization", "data.js.jinja2")
    web_index_template_path = os.path.join(template_dir, "visualization", "node_diagram.html.jinja2")
    sequence_html_template_path = os.path.join(template_dir, "visualization", "sequence_diagram.html.jinja2")

    # Generate visualization for each mode
    for mode_key, deploy_instance in deploy_instances.items():
        # Collect data from the system instance
        data = deploy_instance.collect_instance_data()

        # Create mode-specific output directory
        mode_visualization_dir = os.path.join(visualization_dir, mode_key)

        # Generate diagrams with mode suffix in filename
        filename_base = f"{name}_{mode_key}" if mode_key != "default" else name
        generate_by_template(data, node_dot_template_path, mode_visualization_dir, filename_base + "_node_graph.dot")
        generate_by_template(data, logit_dot_template_path, mode_visualization_dir, filename_base + "_logic_graph.dot")

        # Generate JS data for web visualization
        web_data_dir = os.path.join(visualization_dir, "web", "data")
        generate_by_template(data, web_data_template_path, web_data_dir, f"{mode_key}.js")

        logger.info(f"Generated visualization for mode: {mode_key}")

    # Generate web visualization files
    if deploy_instances:
        web_dir = os.path.join(visualization_dir, "web")
        modes = list(deploy_instances.keys())
        default_mode = "default" if "default" in modes else modes[0]

        # Generate node_diagram.html
        index_data = {
            "modes": modes,
            "default_mode": default_mode
        }
        generate_by_template(index_data, web_index_template_path, web_dir, "node_diagram.html")
        logger.info("Generated web visualization node_diagram.html")

        # Generate sequence_graph.html for each mode
        for mode_key, deploy_instance in deploy_instances.items():
            data = deploy_instance.collect_instance_data()
            filename_base = f"{name}_{mode_key}" if mode_key != "default" else name
            generate_by_template(data, sequence_html_template_path, web_dir, filename_base + "_sequence_graph.html")
            logger.info(f"Generated web visualization sequence_graph.html for mode: {mode_key}")


def generate_by_template(data, template_path, output_dir, output_filename):
    """Generate file from template using the unified template renderer."""
    # Initialize template renderer
    renderer = TemplateRenderer()

    # Get template name from path
    template_name = os.path.basename(template_path)

    # Render template and save to file
    output_path = os.path.join(output_dir, output_filename)
    renderer.render_template_to_file(template_name, output_path, **data)
