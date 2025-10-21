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

from typing import List, Dict, Any, Optional
from dataclasses import dataclass
from pathlib import Path

class ConfigType:
    """Constants for element types."""
    MODULE = "module"
    PIPELINE = "pipeline"
    PARAMETER_SET = "parameter_set"
    ARCHITECTURE = "architecture"

    @classmethod
    def get_all_types(cls) -> List[str]:
        """Get all valid element types."""
        return [cls.MODULE, cls.PIPELINE, cls.PARAMETER_SET, cls.ARCHITECTURE]

@dataclass
class Config:
    """Pure data structure for element configuration."""
    name: str
    full_name: str
    element_type: str
    config: Dict[str, Any]
    file_path: Path
    
    def __post_init__(self):
        """Ensure file_path is a Path object."""
        if isinstance(self.file_path, str):
            self.file_path = Path(self.file_path)

@dataclass
class ModuleConfig(Config):
    """Data structure for module elements."""
    launch: Dict[str, Any] = None
    inputs: List[Dict[str, Any]] = None
    outputs: List[Dict[str, Any]] = None
    parameter_files: Any = None  # Can be dict or list
    configurations: Any = None  # Can be dict or list
    processes: List[Dict[str, Any]] = None

@dataclass
class PipelineConfig(Config):
    """Data structure for pipeline elements."""
    depends: List[str] = None
    nodes: List[Dict[str, Any]] = None
    external_interfaces: Any = None  # Can be dict or list
    connections: List[Dict[str, Any]] = None

@dataclass
class ParameterSetConfig(Config):
    """Data structure for parameter set elements."""
    parameters: Any = None  # Can be dict or list

@dataclass
class ArchitectureConfig(Config):
    """Data structure for architecture elements."""
    modes: List[Dict[str, Any]] = None
    components: List[Dict[str, Any]] = None
    connections: List[Dict[str, Any]] = None