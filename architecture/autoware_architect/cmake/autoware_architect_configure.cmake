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


# collect all yaml files in the architecture directory
# store the list in a shared file
macro(autoware_architect_configure)
  # Usage: autoware_architect_configure() OR autoware_architect_configure(<domain>)
  # Domain defaults to 'shared' when omitted or empty.
  if(ARGC GREATER 1)
    message(FATAL_ERROR "autoware_architect_configure: expects 0 or 1 argument (domain). Got ARGC='${ARGC}' ARGV='${ARGV}'")
  endif()

  # Raw domain (exact user input before normalization)
  set(_ARCH_DOMAIN_RAW "shared")
  if(ARGC EQUAL 1)
    set(_ARCH_DOMAIN_RAW "${ARGV0}")
  endif()

  # Show initial capture
  message(STATUS "autoware_architect_configure: ARGC='${ARGC}' RAW='${_ARCH_DOMAIN_RAW}'")

  # Trim and lowercase for final use
  string(STRIP "${_ARCH_DOMAIN_RAW}" _ARCH_DOMAIN_STRIPPED)
  if("${_ARCH_DOMAIN_STRIPPED}" STREQUAL "")
    set(_ARCH_DOMAIN_STRIPPED "shared")
  endif()
  string(TOLOWER "${_ARCH_DOMAIN_STRIPPED}" _ARCH_DOMAIN_FINAL)

  # Safeguard: if user provided a non-empty, non-shared domain that collapses to 'shared', raise fatal to surface hidden override.
  if(ARGC EQUAL 1 
     AND NOT "${_ARCH_DOMAIN_RAW}" STREQUAL ""
     AND NOT "${_ARCH_DOMAIN_RAW}" STREQUAL "shared"
     AND "${_ARCH_DOMAIN_FINAL}" STREQUAL "shared")
    message(FATAL_ERROR "autoware_architect_configure: Provided domain='${_ARCH_DOMAIN_RAW}' unexpectedly normalized to 'shared'. This indicates an override or logic error. Please inspect the macro call site.")
  endif()

  message(STATUS "autoware_architect_configure: domain raw='${_ARCH_DOMAIN_RAW}' final='${_ARCH_DOMAIN_FINAL}' for package='${PROJECT_NAME}'")

  # Collect all yaml files in the package's architecture directory
  file(GLOB_RECURSE YAML_FILES "${CMAKE_CURRENT_SOURCE_DIR}/architecture/*.yaml")

  # Directory to place per-package manifest resources
  set(resource_dir "${CMAKE_BINARY_DIR}/../autoware_architect/resource")
  file(MAKE_DIRECTORY ${resource_dir})

  # Per-package manifest file
  set(manifest_file "${resource_dir}/${PROJECT_NAME}.yaml")

  # Start (overwrite) manifest
  file(WRITE ${manifest_file} "domain: ${_ARCH_DOMAIN_FINAL}\n")
  file(APPEND ${manifest_file} "architecture_config_files:\n")

  foreach(YAML_FILE ${YAML_FILES})
    # Infer type from filename pattern
    set(file_type "unknown")
    if(YAML_FILE MATCHES ".*\\.module\\.yaml$")
      set(file_type "module")
    elseif(YAML_FILE MATCHES ".*\\.pipeline\\.yaml$")
      set(file_type "pipeline")
    elseif(YAML_FILE MATCHES ".*architecture\\.yaml$")
      set(file_type "architecture")
    endif()

    # Append entry to manifest
  file(APPEND ${manifest_file} "  - path: ${YAML_FILE}\n")
  file(APPEND ${manifest_file} "    type: ${file_type}\n")
  # Item inherits the manifest domain (could be extended per-file later)
  endforeach()

  message(STATUS "autoware_architect: generated per-package manifest ${manifest_file} (domain='${_ARCH_DOMAIN_FINAL}', files: ${YAML_FILES})")
endmacro()
