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
  # Invocation patterns:
  #   autoware_architect_build_deploy(<project> <deployment_file>)
  #   autoware_architect_build_deploy(<project> <domain> <deployment_file>)
  set(_EXTRA_ARGS ${ARGN})
  list(LENGTH _EXTRA_ARGS _EXTRA_LEN)
  if(_EXTRA_LEN EQUAL 1)
    # Only deployment file provided
    list(GET _EXTRA_ARGS 0 _DEPLOYMENT_FILE_NAME)
    set(_DEPLOY_DOMAIN "shared")
  elseif(_EXTRA_LEN EQUAL 2)
    list(GET _EXTRA_ARGS 0 _DEPLOYMENT_FILE_NAME)
    list(GET _EXTRA_ARGS 1 _DEPLOY_DOMAIN)
  else()
    message(FATAL_ERROR "autoware_architect_build_deploy: expected 1 or 2 extra args (deployment_file | domain deployment_file), got ${_EXTRA_LEN}: ${_EXTRA_ARGS}")
  endif()

  if("${_DEPLOYMENT_FILE_NAME}" STREQUAL "")
    message(FATAL_ERROR "autoware_architect_build_deploy: empty deployment file name (args=${_EXTRA_ARGS})")
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

  # run build.py script, without target
  if("${_DEPLOY_DOMAIN}" STREQUAL "")
    set(_DEPLOY_DOMAIN "shared")
  endif()

  add_custom_target(run_build_py_${_DEPLOYMENT_FILE_NAME} ALL
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${ARCHITECT_SOURCE_DIR}:$ENV{PYTHONPATH} python3 -d ${BUILD_PY_SCRIPT} ${DEPLOYMENT_FILE} ${ARCHITECTURE_RESOURCE_DIR} ${OUTPUT_ROOT_DIR} ${_DEPLOY_DOMAIN} > ${LOG_FILE} 2>&1
    COMMENT "Running build.py script from autoware_architect package (domain=${_DEPLOY_DOMAIN}). Check the log file at ${LOG_FILE} for details."
  )

  # add dependencies to trigger the build.py script when building the project
  add_dependencies(${project_name} run_build_py_${_DEPLOYMENT_FILE_NAME})
endmacro()
