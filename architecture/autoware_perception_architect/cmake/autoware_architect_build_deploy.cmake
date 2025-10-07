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

macro(autoware_architect_build_deploy project_name deployment_file)
  set(BUILD_PY_SCRIPT "${CMAKE_BINARY_DIR}/../autoware_perception_architect/script/deployment_process.py")
  set(DEPLOYMENT_FILE "${CMAKE_SOURCE_DIR}/deployment/${deployment_file}.yaml")
  set(ARCHITECTURE_YAML_LIST "${CMAKE_BINARY_DIR}/../autoware_perception_architect/autoware_architect_yaml_filelist.txt")
  set(OUTPUT_ROOT_DIR "${CMAKE_INSTALL_PREFIX}/share/${CMAKE_PROJECT_NAME}/")
  # Use CMAKE_BINARY_DIR to get to workspace root, then navigate to log directory
  get_filename_component(WORKSPACE_ROOT "${CMAKE_BINARY_DIR}/../.." ABSOLUTE)
  set(LOG_FILE "${WORKSPACE_ROOT}/log/latest_build/${CMAKE_PROJECT_NAME}/build_${deployment_file}.log")
  set(ARCHITECT_SOURCE_DIR "${CMAKE_SOURCE_DIR}/../architecture/autoware_perception_architect")

  # run build.py script, without target
  add_custom_target(run_build_py_${deployment_file} ALL
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${ARCHITECT_SOURCE_DIR}:$ENV{PYTHONPATH} python3 -d ${BUILD_PY_SCRIPT} ${DEPLOYMENT_FILE} ${ARCHITECTURE_YAML_LIST} ${OUTPUT_ROOT_DIR} > ${LOG_FILE} 2>&1
    COMMENT "Running build.py script from autoware_perception_architect package. Check the log file at ${LOG_FILE} for details."
  )

  # add dependencies to trigger the build.py script when building the project
  add_dependencies(${project_name} run_build_py_${deployment_file})
endmacro()
