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
  # Validate input parameters
  if("${project_name}" STREQUAL "")
    message(FATAL_ERROR "autoware_architect_build_deploy: project_name is required")
  endif()
  if("${deployment_file}" STREQUAL "")
    message(FATAL_ERROR "autoware_architect_build_deploy: deployment_file is required")
  endif()

  # Set paths with better error checking
  set(BUILD_PY_SCRIPT "${CMAKE_BINARY_DIR}/../autoware_perception_architect/script/build.py")
  set(ARCHITECTURE_YAML_LIST "${CMAKE_BINARY_DIR}/../autoware_perception_architect/autoware_architect_yaml_filelist.txt")
  set(DEPLOYMENT_FILE "${CMAKE_SOURCE_DIR}/deployment/${deployment_file}.yaml")
  set(OUTPUT_ROOT_DIR "${CMAKE_INSTALL_PREFIX}/share/${CMAKE_PROJECT_NAME}/")
  set(LOG_FILE "${CMAKE_BINARY_DIR}/build_${deployment_file}.log")

  # Check if required files exist during configuration
  if(NOT EXISTS "${CMAKE_SOURCE_DIR}/deployment/${deployment_file}.yaml")
    message(WARNING "Deployment file not found: ${CMAKE_SOURCE_DIR}/deployment/${deployment_file}.yaml")
  endif()

  # Create custom target with simplified command structure
  add_custom_target(run_build_py_${deployment_file} ALL
    COMMAND python3 "${BUILD_PY_SCRIPT}" "${DEPLOYMENT_FILE}" "${ARCHITECTURE_YAML_LIST}" "${OUTPUT_ROOT_DIR}"
    DEPENDS "${BUILD_PY_SCRIPT}" "${ARCHITECTURE_YAML_LIST}" "${DEPLOYMENT_FILE}"
    BYPRODUCTS "${OUTPUT_ROOT_DIR}/launchers" "${OUTPUT_ROOT_DIR}/visualization"
    COMMENT "Running build.py script for ${deployment_file}"
    VERBATIM
  )

  # Add proper dependencies
  add_dependencies(${project_name} run_build_py_${deployment_file})

  # Ensure the architecture configure step runs before this
  if(TARGET autoware_architect_configure_target)
    add_dependencies(run_build_py_${deployment_file} autoware_architect_configure_target)
  endif()

  # Show helpful message
  message(STATUS "Configured autoware architect build for deployment: ${deployment_file}")
endmacro()
