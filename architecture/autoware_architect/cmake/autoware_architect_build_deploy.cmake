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

macro(autoware_architect_build_deploy project_name)
  # Supported invocation patterns:
  #   autoware_architect_build_deploy(<project> <deployment_file>)                  # no domains -> shared implied later
  #   autoware_architect_build_deploy(<project> <deployment_file> domainA)          # single domain
  #   autoware_architect_build_deploy(<project> <deployment_file> domainA domainB) # multiple domains
  #   autoware_architect_build_deploy(<project> <deployment_file> domainA;domainB) # semicolon list as single arg
  set(_EXTRA_ARGS ${ARGN})
  list(LENGTH _EXTRA_ARGS _EXTRA_LEN)
  if(_EXTRA_LEN LESS 1)
    message(FATAL_ERROR "autoware_architect_build_deploy: expected at least 1 extra arg (<deployment_file> [domains...]), got ${_EXTRA_LEN}: ${_EXTRA_ARGS}")
  endif()
  list(GET _EXTRA_ARGS 0 _DEPLOYMENT_FILE_NAME)
  set(_DOMAIN_ARGS "")
  if(_EXTRA_LEN GREATER 1)
    list(REMOVE_AT _EXTRA_ARGS 0)
    set(_DOMAIN_ARGS ${_EXTRA_ARGS})
  endif()

  if("${_DEPLOYMENT_FILE_NAME}" STREQUAL "")
    message(FATAL_ERROR "autoware_architect_build_deploy: empty deployment file name")
  endif()
  set(BUILD_PY_SCRIPT "${CMAKE_BINARY_DIR}/../autoware_architect/script/deployment_process.py")
  set(DEPLOYMENT_FILE "${CMAKE_SOURCE_DIR}/deployment/${_DEPLOYMENT_FILE_NAME}.yaml")
  # Pass directory containing per-package manifests
  set(ARCHITECTURE_RESOURCE_DIR "${CMAKE_BINARY_DIR}/../autoware_architect/resource")
  set(OUTPUT_ROOT_DIR "${CMAKE_INSTALL_PREFIX}/share/${CMAKE_PROJECT_NAME}/")
  # Use CMAKE_BINARY_DIR to get to workspace root, then navigate to log directory
  get_filename_component(WORKSPACE_ROOT "${CMAKE_BINARY_DIR}/../.." ABSOLUTE)
  set(LOG_FILE "${WORKSPACE_ROOT}/log/latest_build/${CMAKE_PROJECT_NAME}/build_${_DEPLOYMENT_FILE_NAME}.log")
  set(ARCHITECT_SOURCE_DIR "${CMAKE_SOURCE_DIR}/../architecture/autoware_architect")

  # run build.py script (shared will be appended later if needed)

  add_custom_target(run_build_py_${_DEPLOYMENT_FILE_NAME} ALL
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${ARCHITECT_SOURCE_DIR}:$ENV{PYTHONPATH} python3 -d ${BUILD_PY_SCRIPT} ${DEPLOYMENT_FILE} ${ARCHITECTURE_RESOURCE_DIR} ${OUTPUT_ROOT_DIR} ${_DOMAIN_ARGS} > ${LOG_FILE} 2>&1
    COMMENT "Running build.py script (deployment=${_DEPLOYMENT_FILE_NAME}, domains='${_DOMAIN_ARGS}'). Log: ${LOG_FILE}"
  )

  # add dependencies to trigger the build.py script when building the project
  add_dependencies(${project_name} run_build_py_${_DEPLOYMENT_FILE_NAME})
endmacro()
