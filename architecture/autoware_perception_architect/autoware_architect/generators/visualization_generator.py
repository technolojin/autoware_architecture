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

"""Visualization generator for PlantUML diagrams."""

import logging
from pathlib import Path
from typing import List, Dict, Any, Optional

from jinja2 import Environment, FileSystemLoader, select_autoescape

from ..config import config
from ..models.elements import ElementList, Element
from ..exceptions import VisualizationError

logger = logging.getLogger(__name__)


class VisualizationGenerator:
    """Generator for PlantUML visualization files."""
    
    def __init__(self):
        """Initialize visualization generator."""
        self._jinja_env = None
        self._initialized = False
    
    def _setup_templates(self) -> None:
        """Setup Jinja2 template environment."""
        if self._initialized:
            return
            
        try:
            template_dir = config.template_dir
            if template_dir is None:
                # Set default template directory if not configured
                # When running from build directory, find the template directory
                from pathlib import Path
                script_dir = Path(__file__).parent.parent
                # Check if we're in the build directory structure
                possible_template_dirs = [
                    script_dir.parent / "template",  # From script/autoware_architect/generators to template
                    script_dir.parent.parent / "template",  # Alternative path
                    Path(__file__).parent.parent.parent.parent / "template",  # From script/autoware_architect to build root
                ]
                
                template_dir = None
                for possible_dir in possible_template_dirs:
                    if possible_dir.exists():
                        template_dir = possible_dir
                        break
                
                if template_dir is None:
                    raise VisualizationError("Could not find template directory")
                
            if not template_dir.exists():
                raise VisualizationError(f"Template directory not found: {template_dir}")
            
            self._jinja_env = Environment(
                loader=FileSystemLoader(str(template_dir)),
                autoescape=select_autoescape(['html', 'xml']),
                trim_blocks=True,
                lstrip_blocks=True
            )
            
            self._initialized = True
            logger.debug(f"Initialized Jinja2 environment with templates from: {template_dir}")
            
        except Exception as exc:
            raise VisualizationError(f"Failed to setup template environment: {exc}")
    
    def generate_deployment_visualizations(self, deployment_name: str, element_list: ElementList, 
                                         output_dir: Path) -> List[Path]:
        """Generate all visualization files for a deployment.
        
        Args:
            deployment_name: Name of the deployment
            element_list: List of architecture elements
            output_dir: Base output directory
            
        Returns:
            List of generated visualization file paths
            
        Raises:
            VisualizationError: If generation fails
        """
        # Ensure templates are set up
        self._setup_templates()
        
        try:
            # Create exports directory structure matching expected format
            deployment_output_dir = output_dir / "exports" / f"{deployment_name}" / "visualization"
            deployment_output_dir.mkdir(parents=True, exist_ok=True)
            
            generated_files = []
            
            # Generate each type of diagram
            diagram_types = [
                ("logic_diagram.puml.jinja2", "_logic_graph.puml"),
                ("node_diagram.puml.jinja2", "_node_graph.puml"),
                ("sequence_diagram.puml.jinja2", "_sequence_graph.puml")
            ]
            
            for template_name, suffix in diagram_types:
                output_file = deployment_output_dir / f"{deployment_name}{suffix}"
                self._generate_diagram(
                    template_name=template_name,
                    deployment_name=deployment_name,
                    element_list=element_list,
                    output_file=output_file
                )
                generated_files.append(output_file)
                
            logger.info(f"Generated {len(generated_files)} visualization files for deployment: {deployment_name}")
            return generated_files
            
        except Exception as exc:
            raise VisualizationError(f"Failed to generate visualizations for deployment {deployment_name}: {exc}")
    
    def _generate_diagram(self, template_name: str, deployment_name: str, 
                         element_list: ElementList, output_file: Path) -> None:
        """Generate a single diagram file.
        
        Args:
            template_name: Name of the Jinja2 template file
            deployment_name: Name of the deployment
            element_list: List of architecture elements
            output_file: Output file path
        """
        try:
            # Load template
            template = self._jinja_env.get_template(template_name)
            
            # Create instance data compatible with original templates
            instance_data = self._create_instance_data(element_list)
            
            # Prepare template context (matching original structure)
            context = {
                'name': deployment_name,
                'children': instance_data.get('children', []),
                'links': instance_data.get('links', []),
            }
            
            # Render template
            content = template.render(**context)
            
            # Write output file
            with open(output_file, 'w') as f:
                f.write(content)
            
            logger.debug(f"Generated diagram: {output_file}")
            
        except Exception as exc:
            raise VisualizationError(f"Failed to generate diagram {output_file}: {exc}")
    
    def _create_instance_data(self, element_list: ElementList) -> Dict[str, Any]:
        """Create instance data compatible with original templates.
        
        Args:
            element_list: List of architecture elements
            
        Returns:
            Dictionary with 'children' and 'links' keys compatible with original templates
        """
        children = []
        links = []
        
        # Process modules and pipelines as instances
        for element in element_list.elements:
            if element.type in ['module', 'pipeline']:
                instance = self._create_instance_from_element(element)
                children.append(instance)
                
                # Extract links from pipeline connections
                if element.type == 'pipeline':
                    pipeline_links = self._extract_pipeline_links(element)
                    links.extend(pipeline_links)
        
        return {
            'children': children,
            'links': links
        }
    
    def _create_instance_from_element(self, element: Element) -> Dict[str, Any]:
        """Create an instance data structure from an element.
        
        Args:
            element: Architecture element
            
        Returns:
            Instance dictionary compatible with original templates
        """
        # Create basic instance structure
        instance = {
            'name': element.name,
            'id': element.name.replace('.', '_').replace('-', '_'),
            'element_type': element.type,
            'namespace': [element.name],  # Simplified namespace
            'compute_unit': 'default_compute_unit',  # Default compute unit
            'in_ports': [],
            'out_ports': [],
            'events': [],
            'children': [],
            'links': []
        }
        
        # Extract ports and events based on element type
        if element.type == 'module':
            self._populate_module_instance(instance, element)
        elif element.type == 'pipeline':
            self._populate_pipeline_instance(instance, element)
        
        return instance
    
    def _populate_module_instance(self, instance: Dict[str, Any], element: Element) -> None:
        """Populate module instance with ports and events.
        
        Args:
            instance: Instance dictionary to populate
            element: Module element
        """
        config = element.config
        
        # Create input ports
        inputs = config.get('inputs', [])
        for idx, input_config in enumerate(inputs):
            port = self._create_port('in', input_config, idx, instance['id'])
            instance['in_ports'].append(port)
        
        # Create output ports
        outputs = config.get('outputs', [])
        for idx, output_config in enumerate(outputs):
            port = self._create_port('out', output_config, idx, instance['id'])
            instance['out_ports'].append(port)
        
        # Create basic events (simplified)
        instance['events'] = [
            {
                'id': f"{instance['id']}_process_event",
                'name': 'process',
                'type': 'on_input',
                'frequency': 30,  # Default frequency
                'process_event': True,
                'triggers': []
            }
        ]
    
    def _populate_pipeline_instance(self, instance: Dict[str, Any], element: Element) -> None:
        """Populate pipeline instance with external interfaces.
        
        Args:
            instance: Instance dictionary to populate
            element: Pipeline element
        """
        config = element.config
        
        # Create external interface ports
        external_interfaces = config.get('external_interfaces', {})
        
        # Input interfaces
        inputs = external_interfaces.get('inputs', [])
        for idx, input_config in enumerate(inputs):
            port = self._create_port('in', input_config, idx, instance['id'])
            instance['in_ports'].append(port)
        
        # Output interfaces
        outputs = external_interfaces.get('outputs', [])
        for idx, output_config in enumerate(outputs):
            port = self._create_port('out', output_config, idx, instance['id'])
            instance['out_ports'].append(port)
    
    def _create_port(self, port_type: str, port_config: Dict[str, Any], idx: int, instance_id: str) -> Dict[str, Any]:
        """Create a port data structure.
        
        Args:
            port_type: 'in' or 'out'
            port_config: Port configuration from element config
            idx: Port index
            instance_id: ID of the parent instance
            
        Returns:
            Port dictionary compatible with original templates
        """
        port_name = port_config.get('name', f'port_{idx}')
        topic = port_config.get('topic', [port_name])
        
        # Ensure topic is a list
        if isinstance(topic, str):
            topic = topic.split('/')
        
        # Create event for this port
        event = {
            'id': f"{instance_id}_{port_type}_{idx}_event",
            'name': f"{port_type}_{port_name}",
            'type': 'on_input' if port_type == 'in' else 'to_output',
            'frequency': port_config.get('frequency', 30),
            'triggers': []
        }
        
        return {
            'id': f"{instance_id}_{port_type}_{idx}",
            'name': port_name,
            'topic': topic,
            'msg_type': port_config.get('type', 'unknown'),
            'is_global': port_config.get('global', False),
            'event': event
        }
    
    def _extract_pipeline_links(self, element: Element) -> List[Dict[str, Any]]:
        """Extract links from pipeline connections.
        
        Args:
            element: Pipeline element
            
        Returns:
            List of link dictionaries
        """
        links = []
        config = element.config
        connections = config.get('connections', [])
        
        for idx, connection in enumerate(connections):
            # Create simplified link structure
            link = {
                'from_port': {
                    'id': f"link_{idx}_from",
                    'name': connection.get('from', 'unknown'),
                },
                'to_port': {
                    'id': f"link_{idx}_to", 
                    'name': connection.get('to', 'unknown'),
                },
                'msg_type': connection.get('type', 'unknown')
            }
            links.append(link)
        
        return links


# Create singleton instance
visualization_generator = VisualizationGenerator()
