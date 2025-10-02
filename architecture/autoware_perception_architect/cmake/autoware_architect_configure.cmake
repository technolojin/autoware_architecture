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
  # Define the path to the shared file
  set(shared_file "${CMAKE_BINARY_DIR}/../autoware_perception_architect/autoware_architect_yaml_filelist.txt")
  
  # Create directory if it doesn't exist
  get_filename_component(shared_file_dir "${shared_file}" DIRECTORY)
  file(MAKE_DIRECTORY "${shared_file_dir}")
  
  # Find all YAML files in the architecture directory
  file(GLOB_RECURSE YAML_FILES "${CMAKE_CURRENT_SOURCE_DIR}/architecture/*.yaml")
  
  if(YAML_FILES)
    foreach(YAML_FILE ${YAML_FILES})
      # Read the current content of the shared file
      if(EXISTS ${shared_file})
        file(READ ${shared_file} yaml_files_content)
        string(REPLACE "\n" ";" yaml_files_list ${yaml_files_content})
      else()
        set(yaml_files_list "")
      endif()

      # Check if the yaml file is already in the list
      list(FIND yaml_files_list ${YAML_FILE} index)
      if(index EQUAL -1)
        # Append the new yaml file to the shared file
        file(APPEND ${shared_file} "${YAML_FILE}\n")
        message(STATUS "Added to autoware_architect_yaml_file: ${YAML_FILE}")
      else()
        message(STATUS "Already registered: ${YAML_FILE}")
      endif()
    endforeach()
  else()
    message(STATUS "No YAML files found in ${CMAKE_CURRENT_SOURCE_DIR}/architecture/")
  endif()
  
  # Create a configure target that other targets can depend on
  if(NOT TARGET autoware_architect_configure_target)
    add_custom_target(autoware_architect_configure_target
      DEPENDS "${shared_file}"
      COMMENT "Autoware architect configuration completed"
    )
  endif()
  
  # Create a custom command to regenerate the file list if any YAML files change
  if(YAML_FILES)
    add_custom_command(
      OUTPUT "${shared_file}"
      DEPENDS ${YAML_FILES}
      COMMAND ${CMAKE_COMMAND} -E echo "Regenerating YAML file list..."
      COMMENT "Updating autoware architect YAML file list"
    )
  endif()
  
  message(STATUS "Autoware architect configured for ${CMAKE_PROJECT_NAME}")
endmacro()
