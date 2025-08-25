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

"""Custom exceptions for the autoware architecture system."""


class ArchitectureError(Exception):
    """Base exception for architecture-related errors."""
    pass


class ModuleConfigurationError(ArchitectureError):
    """Exception raised for module configuration errors."""
    pass


class PipelineConfigurationError(ArchitectureError):
    """Exception raised for pipeline configuration errors."""
    pass


class ParameterConfigurationError(ArchitectureError):
    """Exception raised for parameter configuration errors."""
    pass


class DeploymentError(ArchitectureError):
    """Exception raised for deployment errors."""
    pass


class ValidationError(ArchitectureError):
    """Exception raised for validation errors."""
    pass
