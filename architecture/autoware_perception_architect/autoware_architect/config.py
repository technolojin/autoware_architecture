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

"""Configuration management for the autoware architecture system."""

import os
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


@dataclass
class ArchitectureConfig:
    """Configuration class for the architecture system."""
    
    debug_mode: bool = False
    layer_limit: int = 50
    output_format: str = "xml"
    log_level: str = "INFO"
    cache_enabled: bool = True
    max_cache_size: int = 128
    template_dir: Optional[Path] = None
    
    @classmethod
    def from_env(cls) -> 'ArchitectureConfig':
        """Create configuration from environment variables."""
        return cls(
            debug_mode=os.getenv('AUTOWARE_ARCHITECT_DEBUG', 'false').lower() == 'true',
            layer_limit=int(os.getenv('AUTOWARE_ARCHITECT_LAYER_LIMIT', '50')),
            output_format=os.getenv('AUTOWARE_ARCHITECT_OUTPUT_FORMAT', 'xml'),
            log_level=os.getenv('AUTOWARE_ARCHITECT_LOG_LEVEL', 'INFO'),
            cache_enabled=os.getenv('AUTOWARE_ARCHITECT_CACHE_ENABLED', 'true').lower() == 'true',
            max_cache_size=int(os.getenv('AUTOWARE_ARCHITECT_MAX_CACHE_SIZE', '128')),
            template_dir=Path(template_dir) if (template_dir := os.getenv('AUTOWARE_ARCHITECT_TEMPLATE_DIR')) else None
        )
    
    def setup_logging(self) -> logging.Logger:
        """Setup logging based on configuration."""
        logger = logging.getLogger('autoware_architect')
        
        # Clear existing handlers
        logger.handlers.clear()
        
        # Create handler
        handler = logging.StreamHandler()
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        
        # Set level
        level = getattr(logging, self.log_level.upper(), logging.INFO)
        if self.debug_mode:
            level = logging.DEBUG
        logger.setLevel(level)
        
        return logger


# Global configuration instance
config = ArchitectureConfig.from_env()
