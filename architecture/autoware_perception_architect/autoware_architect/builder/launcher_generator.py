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
from .instances import Instance, DeploymentInstance


def generate_pipeline_launch_file(instance: Instance, output_dir: str):
    if instance.element_type == "architecture":
        for child in instance.children.values():
            # Split component name by slashes and use only the resulting parts as path
            name_parts = child.name.split('/')
            # Use the base namespace + split name parts
            base_namespace = child.namespace[:-1] if child.namespace else []  # Remove the component name from namespace
            path_parts = base_namespace + name_parts
            path = os.path.join(output_dir, *path_parts)
            generate_pipeline_launch_file(child, path)
    elif instance.element_type == "pipeline":
        # generate current pipeline launch file
        pipeline_launch_dict : dict = {}
        pipeline_launch_dict["pipeline_name"] = instance.name
        # 1. define interfaces(topics) from link manager
        # external interfaces, internal interfaces
        # 2. define child nodes
        # package, input, output
        # 5. export file
        # for test, just create an empty launch file
        os.makedirs(output_dir, exist_ok=True)
        
        # Use only the last part of the name for the filename (after the last slash)
        launch_filename = instance.name.split('/')[-1] + ".launch.xml"
        with open(os.path.join(output_dir, launch_filename), "w") as f:
            f.write("<launch>\n")
            f.write(instance.namespace_str)
            f.write("</launch>\n")
        # recursively call children pipelines
        for child in instance.children.values():
            # For pipeline children, simply use the child's name directly
            # The output_dir already contains the correct parent path
            child_name_parts = child.name.split('/')
            child_path = os.path.join(output_dir, *child_name_parts)
            generate_pipeline_launch_file(child, child_path)
    elif instance.element_type == "module":
        # module launch files are already generated on each package build process
        return
    else:
        raise ValueError(f"Invalid element type: {instance.element_type}")
