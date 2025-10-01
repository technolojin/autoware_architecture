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

from enum import Enum
from logging import getLogger
from typing import Dict
from typing import List

from utils import load_config_yaml

logger = getLogger("__name__")


class ElementType(Enum):
    MODULE = "module"
    PIPELINE = "pipeline"
    PARAMETER_SET = "parameter_set"
    ARCHITECTURE = "architecture"


def element_name_decode(element_name) -> (str, str):
    if "." not in element_name:
        raise ValueError(f"Invalid element name: '{element_name}'")

    name, type_str = element_name.split(".", 1)
    try:
        return name, ElementType(type_str)
    except ValueError:
        raise ValueError(f"Invalid element type: '{type_str}'")


class Element:
    def __init__(self, config_yaml_dir):
        self.config_yaml_dir = config_yaml_dir
        self.config_yaml = load_config_yaml(config_yaml_dir)

        if "name" not in self.config_yaml:
            raise ValueError(
                f"Field 'name' is required in element configuration. File {self.config_yaml_dir}"
            )

        self.full_name = self.config_yaml.get("name")
        self.name, self.type = element_name_decode(self.full_name)
        self.check_config()

    def check_config(self) -> bool:
        return True


class ModuleElement(Element):
    def check_config(self) -> bool:
        if self.type != ElementType.MODULE:
            raise ValueError(
                f"Invalid element name: '{self.full_name}'. File {self.config_yaml_dir}"
            )
        required_fields = [
            "name",
            "launch",
            "inputs",
            "outputs",
            "parameters",
            "configurations",
            "processes",
        ]
        for field in required_fields:
            if field not in self.config_yaml:
                raise ValueError(
                    f"Field '{field}' is required in module configuration. File {self.config_yaml_dir}"
                )
        return True


class PipelineElement(Element):
    def check_config(self) -> bool:
        if self.type != ElementType.PIPELINE:
            raise ValueError(
                f"Invalid element name: '{self.full_name}'. File {self.config_yaml_dir}"
            )
        required_fields = [
            "name",
            "depends",
            "nodes",
            "external_interfaces",
            "connections",
            "parameters",
            "configurations",
        ]
        for field in required_fields:
            if field not in self.config_yaml:
                raise ValueError(
                    f"Field '{field}' is required in pipeline configuration. File {self.config_yaml_dir}"
                )
        return True


class ParameterSetElement(Element):
    def check_config(self) -> bool:
        if self.type != ElementType.PARAMETER_SET:
            raise ValueError(
                f"Invalid element name: '{self.full_name}'. File {self.config_yaml_dir}"
            )
        required_fields = ["name", "parameters"]
        for field in required_fields:
            if field not in self.config_yaml:
                raise ValueError(
                    f"Field '{field}' is required in parameter_set configuration. File {self.config_yaml_dir}"
                )
        return True


class ArchitectureElement(Element):
    def check_config(self) -> bool:
        if self.type != ElementType.ARCHITECTURE:
            raise ValueError(
                f"Invalid element name: '{self.full_name}'. File {self.config_yaml_dir}"
            )
        required_fields = ["name", "components", "connections"]
        for field in required_fields:
            if field not in self.config_yaml:
                raise ValueError(
                    f"Field '{field}' is required in architecture configuration. File {self.config_yaml_dir}"
                )
        return True


ELEMENT_TYPES = {
    ElementType.MODULE: ModuleElement,
    ElementType.PIPELINE: PipelineElement,
    ElementType.PARAMETER_SET: ParameterSetElement,
    ElementType.ARCHITECTURE: ArchitectureElement,
}


class ElementList:
    def __init__(self, config_yaml_file_dirs: List[str]):
        self.elements: Dict[str, Element] = {}
        self._fill_list(config_yaml_file_dirs)

    def _fill_list(self, config_yaml_file_dirs: List[str]):
        for config_yaml_file_dir in config_yaml_file_dirs:
            logger.debug(f"ElementList: Loading {config_yaml_file_dir}")
            _, elem_type = element_name_decode(load_config_yaml(config_yaml_file_dir)["name"])
            element = ELEMENT_TYPES[elem_type](config_yaml_file_dir)

            if element.full_name in self.elements:
                raise ValueError(
                    f"Element {element.full_name} is already in the list: \n to be added {element.config_yaml_dir}\n exist {self.elements[element.full_name].config_yaml_dir}"
                )
            self.elements[element.full_name] = element

    def get_elements_by_type(self, element_type: ElementType) -> List[Element]:
        return [elem for elem in self.elements.values() if elem.type == element_type]
