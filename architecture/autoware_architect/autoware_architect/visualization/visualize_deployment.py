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
from ..template_utils import TemplateRenderer

logger = logging.getLogger(__name__)


def visualize_deployment(deploy_data: Dict[str, Dict], name: str, visualization_dir: str):
    """Generate visualization files for deployment data.

    Args:
        deploy_data: Dictionary mapping mode names to deployment data dictionaries
        name: Base name for the deployment
        visualization_dir: Directory to output visualization files
    """
    # load the template file
    template_dir = os.path.join(os.path.dirname(__file__), "../template")
    node_dot_template_path = os.path.join(template_dir, "node_diagram.dot.jinja2")
    logit_dot_template_path = os.path.join(template_dir, "logic_diagram.dot.jinja2")

    # Web visualization templates
    web_data_template_path = os.path.join(template_dir, "visualization", "data", "node_diagram_data.js.jinja2")
    web_index_template_path = os.path.join(template_dir, "visualization", "node_diagram.html.jinja2")
    sequence_html_template_path = os.path.join(template_dir, "visualization", "sequence_diagram.html.jinja2")
    sequence_mermaid_template_path = os.path.join(template_dir, "visualization", "data", "sequence_diagram_mermaid.jinja2")
    sequence_data_template_path = os.path.join(template_dir, "visualization", "data", "sequence_diagram_data.js.jinja2")

    # Module templates for overview page
    node_module_template_path = os.path.join(template_dir, "visualization", "node_diagram.js.jinja2")
    sequence_module_template_path = os.path.join(template_dir, "visualization", "sequence_diagram.js.jinja2")
    overview_template_path = os.path.join(template_dir, "visualization", "deployment_overview.html.jinja2")

    # Generate visualization for each mode
    for mode_key, data in deploy_data.items():

        # Create mode-specific output directory
        mode_visualization_dir = os.path.join(visualization_dir, mode_key)

        # Generate diagrams with mode suffix in filename
        filename_base = f"{name}_{mode_key}" if mode_key != "default" else name
        generate_by_template(data, node_dot_template_path, mode_visualization_dir, filename_base + "_node_graph.dot")
        generate_by_template(data, logit_dot_template_path, mode_visualization_dir, filename_base + "_logic_graph.dot")

        # Generate JS data for web visualization
        web_data_dir = os.path.join(visualization_dir, "web", "data")
        node_data_with_mode = {**data, "mode": mode_key}
        generate_by_template(node_data_with_mode, web_data_template_path, web_data_dir, f"{mode_key}_node_diagram.js")

        # Generate sequence diagram Mermaid syntax and data
        renderer = TemplateRenderer()
        mermaid_syntax = renderer.render_template("visualization/data/sequence_diagram_mermaid.jinja2", **data)
        sequence_data = {
            "mode": mode_key,
            "mermaid_syntax": mermaid_syntax
        }
        generate_by_template(sequence_data, sequence_data_template_path, web_data_dir, f"{mode_key}_sequence_diagram.js")

        logger.info(f"Generated visualization for mode: {mode_key}")

    # Generate web visualization files
    if deploy_data:
        web_dir = os.path.join(visualization_dir, "web")
        modes = list(deploy_data.keys())
        default_mode = "default" if "default" in modes else modes[0]

        # Generate node_diagram.html
        index_data = {
            "modes": modes,
            "default_mode": default_mode
        }
        generate_by_template(index_data, web_index_template_path, web_dir, "node_diagram.html")
        logger.info("Generated web visualization node_diagram.html")

        # Generate sequence_diagram.html (single file with mode selector)
        sequence_index_data = {
            "name": name,
            "modes": modes,
            "default_mode": default_mode
        }
        generate_by_template(sequence_index_data, sequence_html_template_path, web_dir, "sequence_diagram.html")
        logger.info("Generated web visualization sequence_diagram.html")

        # Generate module JS files for overview page
        module_data = {
            "modes": modes,
            "default_mode": default_mode
        }
        generate_by_template(module_data, node_module_template_path, web_dir, "node_diagram.js")
        logger.info("Generated node diagram module: node_diagram.js")

        generate_by_template(module_data, sequence_module_template_path, web_dir, "sequence_diagram.js")
        logger.info("Generated sequence diagram module: sequence_diagram.js")

        # Generate overview HTML file
        overview_data = {
            "deployment_name": name,
            "package_name": name,  # Using name as package name for now
            "available_modes": modes,
            "available_diagram_types": ["node_diagram", "sequence_diagram"],
            "default_mode": default_mode,
            "default_diagram_type": "node_diagram"
        }
        generate_by_template(overview_data, overview_template_path, web_dir, f"{name}_overview.html")
        logger.info(f"Generated deployment overview: {name}_overview.html")


def generate_by_template(data, template_path, output_dir, output_filename):
    """Generate file from template using the unified template renderer."""
    # Initialize template renderer
    renderer = TemplateRenderer()

    # Get template name from path - handle subdirectories
    # Find the relative path from template_dir
    template_dir = os.path.join(os.path.dirname(__file__), "../template")
    try:
        template_name = os.path.relpath(template_path, template_dir)
    except ValueError:
        # If relative path fails, fall back to basename
        template_name = os.path.basename(template_path)

    # Render template and save to file
    output_path = os.path.join(output_dir, output_filename)
    renderer.render_template_to_file(template_name, output_path, **data)
