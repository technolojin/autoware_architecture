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

from typing import List

import yaml

debug_mode = True

list_of_element_types = ["module", "pipeline", "parameter_set", "architecture"]


def load_config_yaml(config_yaml_dir):
    with open(config_yaml_dir, "r") as stream:
        try:
            config_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return
    return config_yaml


def element_name_decode(element_name) -> (str, str):
    # example of full_name: "ObjectDetector.module"
    if "." not in element_name:
        raise ValueError(f"Invalid element name: '{element_name}'")

    splitted = element_name.split(".")
    if len(splitted) < 2:
        raise ValueError(f"Invalid element name: '{element_name}'")

    element_name = splitted[0]  # example: "ObjectDetector"
    element_type = splitted[1]  # example: "module"

    # Check the element type
    if element_type not in list_of_element_types:
        raise ValueError(f"Invalid element type: '{element_type}'")

    return element_name, element_type


# classes for architecture configuration

class Element:
    def __init__(self, config_yaml_dir):
        self.config_yaml_dir = config_yaml_dir
        self.config_yaml = load_config_yaml(config_yaml_dir)

        # Check the name field
        if "name" not in self.config_yaml:
            print(f"Field 'name' is required in element configuration. File {self.config_yaml_dir}")
            return False

        self.full_name = self.config_yaml.get("name")
        self.name, self.type = element_name_decode(self.full_name)

        # Check the element
        if self.type not in list_of_element_types:
            raise ValueError(f"Invalid element type: '{self.type}'. File {self.config_yaml_dir}")
        self.check_config()

    def check_config(self) -> bool:
        return True

class ModuleElement(Element):
    def check_config(self) -> bool:
        # Check the name is [name].module
        if self.type != "module":
            raise ValueError(
                f"Invalid element name: '{self.full_name}'. File {self.config_yaml_dir}"
            )
            return False

        # Check the required field
        required_field = [
            "name",
            "launch",
            "inputs",
            "outputs",
            "parameters",
            "configurations",
            "processes",
        ]
        for field in required_field:
            if field not in self.config_yaml:
                raise ValueError(
                    f"Field '{field}' is required in module configuration. Please check file {self.config_yaml_dir}"
                )
                return False

        # All checks passed
        return True
    
class PipelineElement(Element):
    def check_config(self) -> bool:
        # Check the name is [name].pipeline
        if self.type != "pipeline":
            raise ValueError(
                f"Invalid element name: '{self.full_name}'. File {self.config_yaml_dir}"
            )
            return False

        # Check the required field
        required_field = [
            "name",
            "depends",
            "nodes",
            "external_interfaces",
            "connections",
            "parameters",
            "configurations",
        ]

        for field in required_field:
            if field not in self.config_yaml:
                raise ValueError(
                    f"Field '{field}' is required in pipeline configuration. Please check file {self.config_yaml_dir}"
                )
                return False

        # All checks passed
        return True

class ParameterSetElement(Element):
    def check_config(self) -> bool:
        # Check the name is [name].parameter_set
        if self.type != "parameter_set":
            raise ValueError(
                f"Invalid element name: '{self.full_name}'. File {self.config_yaml_dir}"
            )
            return False

        # Check the required fields
        required_field = [
            "name",
            "parameters",
        ]

        for field in required_field:
            if field not in self.config_yaml:
                raise ValueError(
                    f"Field '{field}' is required in parameter_set configuration. Please check file {self.config_yaml_dir}"
                )
                return False

        # All checks passed
        return True

class ArchitectureElement(Element):
    def check_config(self) -> bool:
        if self.type != "architecture":
            raise ValueError(
                f"Invalid element name: '{self.full_name}'. File {self.config_yaml_dir}"
            )
            return False

        # Check the required field
        required_field = [
            "name",
            "components",
            "connections",
        ]

        for field in required_field:
            if field not in self.config_yaml:
                raise ValueError(
                    f"Field '{field}' is required in architecture configuration. Please check file {self.config_yaml_dir}"
                )
                return False

        # All checks passed
        return True


class ElementList:
    def __init__(self, config_yaml_file_dirs: List[str]):
        self.elements: List[Element] = []
        try:
            self._fill_list(config_yaml_file_dirs)
        except ValueError as e:
            print(e, config_yaml_file_dirs)
            raise

    def _fill_list(self, config_yaml_file_dirs: List[str]):
        for config_yaml_file_dir in config_yaml_file_dirs:
            if debug_mode:
                print(f"ElementList fill_list: Loading {config_yaml_file_dir}")
            element = Element(config_yaml_file_dir)
            # check if the element is already in the list
            for e in self.elements:
                if e.full_name == element.full_name:
                    raise ValueError(
                        f"Element {e.full_name} is already in the list: \n to be added {element.config_yaml_dir}\n exist {e.config_yaml_dir}"
                    )
            # add the element to the list
            self.elements.append(element)

    def get_module_list(self):
        return [element.config_yaml_dir for element in self.elements if element.type == "module"]

    def get_pipeline_list(self):
        return [element.config_yaml_dir for element in self.elements if element.type == "pipeline"]

    def get_parameter_set_list(self):
        return [
            element.config_yaml_dir for element in self.elements if element.type == "parameter_set"
        ]

    def get_architecture_list(self):
        return [
            element.config_yaml_dir for element in self.elements if element.type == "architecture"
        ]


class ModuleList:
    def __init__(self, module_list: List[Element]):
        self.list: List[ModuleElement] = []
        for module in module_list:
            self.list.append(ModuleElement(module))

    def get(self, module_name):
        for module in self.list:
            if module.name == module_name:
                return module
        raise ValueError(f"ModuleList: Module not found: {module_name}")


class PipelineList:
    def __init__(self, pipeline_list: List[Element]):
        self.list: List[PipelineElement] = []
        for pipeline in pipeline_list:
            self.list.append(PipelineElement(pipeline))

    def get(self, pipeline_name):
        for pipeline in self.list:
            if pipeline.name == pipeline_name:
                return pipeline
        # if not found, print the list of pipelines
        list_text = ""
        for pipeline in self.list:
            list_text += f"{pipeline.name}\n"
        raise ValueError(
            f"PipelineList: Pipeline not found: {pipeline_name}, available pipelines are:\n{list_text}"
        )


class ParameterSetList:
    def __init__(self, parameter_set_list: List[Element]):
        self.list: List[ParameterSetElement] = []
        for parameter_set in parameter_set_list:
            self.list.append(ParameterSetElement(parameter_set))

    def get(self, parameter_set_name):
        for parameter_set in self.list:
            if parameter_set.name == parameter_set_name:
                return parameter_set
        # if not found, print the list of parameter sets
        list_text = ""
        for parameter_set in self.list:
            list_text += f"{parameter_set.name}\n"
        raise ValueError(
            f"ParameterSetList: Parameter set not found: {parameter_set_name}, available parameter sets are:\n{list_text}"
        )


class ArchitectureList:
    def __init__(self, architecture_list: List[Element]):
        self.list: List[ArchitectureElement] = []
        for architecture in architecture_list:
            self.list.append(ArchitectureElement(architecture))

    def get(self, architecture_name):
        # if the name is full name, decode it
        if "." in architecture_name:
            architecture_name, _ = element_name_decode(architecture_name)

        for architecture in self.list:
            if architecture.name == architecture_name:
                return architecture
        # if not found, print the list of architectures
        list_text = ""
        for architecture in self.list:
            list_text += f"{architecture.name}\n"
        raise ValueError(
            f"ArchitectureList: Architecture not found: {architecture_name}, available architectures are:\n{list_text}"
        )
