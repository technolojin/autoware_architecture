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

from typing import Dict, Any
from pathlib import Path
import logging

from ..parsers.yaml_parser import yaml_parser
from .data_validator import ValidatorFactory, element_name_decode
from ..models.config import Config, ModuleConfig, PipelineConfig, ParameterSetConfig, ArchitectureConfig, ConfigType
from ..exceptions import ValidationError

logger = logging.getLogger(__name__)

class ConfigParser:
    """Parser for element configuration files."""
    
    def __init__(self):
        self.validator_factory = ValidatorFactory()
    
    def parse_element_file(self, config_yaml_path: str) -> Config:
        """Parse an element configuration file."""
        file_path = Path(config_yaml_path)
        # get element type from file name 
        # file/path/to/<element_name>.<element_type>.yaml
        file_element_name, file_element_type = element_name_decode(file_path.stem)

        # Load configuration
        config = self._load_config(file_path)
        
        # Parse element name and type
        full_name = config.get("name")
        element_name, element_type = element_name_decode(full_name)

        if element_name != file_element_name:
            raise ValidationError(
                f"Config name '{element_name}' does not match file name '{file_element_name}'. File: {file_path}"
            )
        
        # Validate configuration
        validator = self.validator_factory.get_validator(element_type)
        validator.validate_all(config, element_type, file_element_type, str(file_path))
        
        # Create appropriate data structure
        return self._create_element_data(element_name, full_name, element_type, config, file_path)
    
    def _load_config(self, file_path: Path) -> Dict[str, Any]:
        """Load YAML configuration file."""
        try:
            return yaml_parser.load_config(str(file_path))
        except Exception as e:
            logger.error(f"Failed to load config from {file_path}: {e}")
            raise ValidationError(f"Error parsing YAML file {file_path}: {e}")

    def _create_element_data(self, element_name: str, full_name: str, element_type: str, 
                           config: Dict[str, Any], file_path: Path) -> Config:
        """Create appropriate data structure based on element type."""
        base_data = {
            'name': element_name,
            'full_name': full_name,
            'element_type': element_type,
            'config': config,
            'file_path': file_path
        }
        
        if element_type == ConfigType.MODULE:
            return ModuleConfig(
                **base_data,
                launch=config.get('launch'),
                inputs=config.get('inputs'),
                outputs=config.get('outputs'),
                parameters=config.get('parameters'),
                configurations=config.get('configurations'),
                processes=config.get('processes')
            )
        elif element_type == ConfigType.PIPELINE:
            return PipelineConfig(
                **base_data,
                depends=config.get('depends'),
                nodes=config.get('nodes'),
                external_interfaces=config.get('external_interfaces'),
                connections=config.get('connections'),
                parameters=config.get('parameters'),
                configurations=config.get('configurations')
            )
        elif element_type == ConfigType.PARAMETER_SET:
            return ParameterSetConfig(
                **base_data,
                parameters=config.get('parameters')
            )
        elif element_type == ConfigType.ARCHITECTURE:
            return ArchitectureConfig(
                **base_data,
                components=config.get('components'),
                connections=config.get('connections')
            )
        else:
            raise ValidationError(f"Unknown element type: {element_type}")