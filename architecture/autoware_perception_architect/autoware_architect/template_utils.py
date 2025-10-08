"""
Template utilities for consistent Jinja2 template rendering across the project.
"""
import os
from jinja2 import Environment, FileSystemLoader


class TemplateRenderer:
    """Unified template rendering utility."""
    
    def __init__(self, template_dir: str = None):
        """
        Initialize the template renderer.
        
        Args:
            template_dir: Directory containing template files. If None, uses default.
        """
        if template_dir is None:
            # Default template directory relative to this file
            self.template_dir = os.path.join(os.path.dirname(__file__), "../template")
        else:
            self.template_dir = template_dir
            
        # Setup Jinja2 environment with consistent settings
        self.env = Environment(
            loader=FileSystemLoader(self.template_dir),
            trim_blocks=True,           # Remove newlines after block tags
            lstrip_blocks=True,         # Remove leading whitespace before blocks
            keep_trailing_newline=True, # Preserve final newline
            newline_sequence='\n',      # Consistent line endings
            autoescape=False            # Don't auto-escape (for non-HTML templates)
        )
    
    def render_template(self, template_name: str, **kwargs) -> str:
        """
        Render a template with the given data.
        
        Args:
            template_name: Name of the template file
            **kwargs: Template variables
            
        Returns:
            Rendered template content
        """
        template = self.env.get_template(template_name)
        return template.render(**kwargs)
    
    def render_template_to_file(self, template_name: str, output_path: str, **kwargs) -> None:
        """
        Render a template and save to file.
        
        Args:
            template_name: Name of the template file
            output_path: Output file path
            **kwargs: Template variables
        """
        content = self.render_template(template_name, **kwargs)
        
        # Ensure output directory exists
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        # Remove existing file if it exists
        if os.path.exists(output_path):
            os.remove(output_path)
            
        # Write content to file
        with open(output_path, "w") as f:
            f.write(content)
