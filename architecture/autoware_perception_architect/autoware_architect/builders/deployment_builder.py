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

"""Deployment builder for orchestrating the build process."""

import logging
from pathlib import Path
from typing import Union, List, Optional

from ..models.elements import ElementList
from ..parsers.yaml_parser import yaml_parser
from ..generators.launcher_generator import launcher_generator
from ..config import config
from ..exceptions import DeploymentError, ValidationError

logger = logging.getLogger(__name__)


class DeploymentBuilder:
    """Builder class for creating and managing deployments."""
    
    def __init__(self):
        """Initialize deployment builder."""
        self._deployment_config = None
        self._element_list = None
        self._output_dir = None
        self._deployment_name = None
    
    def with_deployment_config(self, config_path: Union[str, Path]) -> 'DeploymentBuilder':
        """Set deployment configuration file.
        
        Args:
            config_path: Path to deployment configuration YAML file
            
        Returns:
            Self for method chaining
        """
        try:
            self._deployment_config = yaml_parser.load_config(config_path)
            self._deployment_name = self._deployment_config.get("name", "unknown")
            logger.info(f"Loaded deployment configuration: {self._deployment_name}")
        except Exception as exc:
            raise DeploymentError(f"Failed to load deployment configuration from {config_path}: {exc}")
        
        return self
    
    def with_element_list_file(self, list_file_path: Union[str, Path]) -> 'DeploymentBuilder':
        """Set element list from file containing paths to element configurations.
        
        Args:
            list_file_path: Path to text file containing element configuration paths
            
        Returns:
            Self for method chaining
        """
        try:
            # Load element configuration paths
            configs = yaml_parser.load_config_list(list_file_path)
            config_paths = list(configs.keys())
            
            # Create element list
            self._element_list = ElementList(config_paths)
            logger.info(f"Loaded {len(self._element_list.elements)} elements")
        except Exception as exc:
            raise DeploymentError(f"Failed to load element list from {list_file_path}: {exc}")
        
        return self
    
    def with_element_paths(self, config_paths: List[Union[str, Path]]) -> 'DeploymentBuilder':
        """Set element list from configuration paths.
        
        Args:
            config_paths: List of paths to element configuration files
            
        Returns:
            Self for method chaining
        """
        try:
            self._element_list = ElementList(config_paths)
            logger.info(f"Loaded {len(self._element_list.elements)} elements")
        except Exception as exc:
            raise DeploymentError(f"Failed to load elements from paths: {exc}")
        
        return self
    
    def with_output_dir(self, output_dir: Union[str, Path]) -> 'DeploymentBuilder':
        """Set output directory for generated files.
        
        Args:
            output_dir: Path to output directory
            
        Returns:
            Self for method chaining
        """
        self._output_dir = Path(output_dir)
        self._output_dir.mkdir(parents=True, exist_ok=True)
        logger.info(f"Set output directory: {self._output_dir}")
        return self
    
    def validate(self) -> 'DeploymentBuilder':
        """Validate the builder configuration.
        
        Returns:
            Self for method chaining
            
        Raises:
            DeploymentError: If configuration is invalid
        """
        if not self._deployment_config:
            raise DeploymentError("Deployment configuration not set")
        
        if not self._element_list:
            raise DeploymentError("Element list not set")
        
        if not self._output_dir:
            raise DeploymentError("Output directory not set")
        
        logger.debug("Deployment configuration validated")
        return self
    
    def build(self) -> 'Deployment':
        """Build the deployment.
        
        Returns:
            Deployment instance
            
        Raises:
            DeploymentError: If build fails
        """
        self.validate()
        
        try:
            deployment = Deployment(
                config=self._deployment_config,
                element_list=self._element_list,
                output_dir=self._output_dir
            )
            
            logger.info(f"Built deployment: {self._deployment_name}")
            return deployment
            
        except Exception as exc:
            raise DeploymentError(f"Failed to build deployment: {exc}")


class Deployment:
    """Represents a complete deployment configuration."""
    
    def __init__(self, config: dict, element_list: ElementList, output_dir: Path):
        """Initialize deployment.
        
        Args:
            config: Deployment configuration
            element_list: List of architecture elements
            output_dir: Output directory for generated files
        """
        self.config = config
        self.element_list = element_list
        self.output_dir = output_dir
        self.name = config.get("name", "unknown")
        
        # Setup logging
        self.logger = config.setup_logging()
        
        logger.info(f"Created deployment: {self.name}")
    
    def generate_launchers(self, executable_name: str = "default_executable") -> List[Path]:
        """Generate launcher files for all modules.
        
        Args:
            executable_name: Name of the executable for modules
            
        Returns:
            List of generated launcher file paths
        """
        logger.info("Generating launcher files...")
        
        modules = self.element_list.get_modules()
        if not modules:
            logger.warning("No modules found for launcher generation")
            return []
        
        # Create launchers subdirectory
        launcher_dir = self.output_dir / "launchers"
        launcher_dir.mkdir(parents=True, exist_ok=True)
        
        # Generate launcher files
        generated_files = launcher_generator.generate_batch_launchers(
            modules=modules,
            executable_name=executable_name,
            output_dir=launcher_dir
        )
        
        logger.info(f"Generated {len(generated_files)} launcher files")
        return generated_files
    
    def generate_visualization(self) -> Optional[Path]:
        """Generate system visualization.
        
        Returns:
            Path to generated visualization file, or None if not implemented
        """
        logger.info("Generating system visualization...")
        # TODO: Implement visualization generation
        logger.warning("Visualization generation not yet implemented")
        return None
    
    def generate_system_monitor(self) -> Optional[Path]:
        """Generate system monitor configuration.
        
        Returns:
            Path to generated monitor configuration, or None if not implemented
        """
        logger.info("Generating system monitor configuration...")
        # TODO: Implement system monitor generation
        logger.warning("System monitor generation not yet implemented")
        return None
    
    def build_all(self, executable_name: str = "default_executable") -> dict:
        """Build all deployment artifacts.
        
        Args:
            executable_name: Name of the executable for modules
            
        Returns:
            Dictionary containing paths to generated artifacts
        """
        logger.info(f"Building all artifacts for deployment: {self.name}")
        
        artifacts = {
            "launchers": self.generate_launchers(executable_name),
            "visualization": self.generate_visualization(),
            "system_monitor": self.generate_system_monitor(),
        }
        
        logger.info("Deployment build completed")
        return artifacts
