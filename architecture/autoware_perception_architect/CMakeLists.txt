cmake_minimum_required(VERSION 3.14)
project(autoware_perception_architect)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

ament_package(
  CONFIG_EXTRAS "autoware_architect-extras.cmake"
)

# Copy the script to the build directory, all of them
# also template files and the autoware_architect package
file(GLOB_RECURSE SCRIPT_FILES ${CMAKE_SOURCE_DIR}/script/*.py)
file(GLOB_RECURSE TEMPLATE_FILES_J2 ${CMAKE_SOURCE_DIR}/template/*.j2)
file(GLOB_RECURSE TEMPLATE_FILES_JINJA2 ${CMAKE_SOURCE_DIR}/template/*.jinja2)
file(GLOB_RECURSE AUTOWARE_ARCHITECT_FILES ${CMAKE_SOURCE_DIR}/autoware_architect/*.py)

foreach(SCRIPT_FILE ${SCRIPT_FILES})
  get_filename_component(SCRIPT_FILE_NAME ${SCRIPT_FILE} NAME)
  configure_file(${SCRIPT_FILE} ${CMAKE_BINARY_DIR}/script/${SCRIPT_FILE_NAME} COPYONLY)
endforeach()

foreach(TEMPLATE_FILE ${TEMPLATE_FILES_J2})
  get_filename_component(TEMPLATE_FILE_NAME ${TEMPLATE_FILE} NAME)
  configure_file(${TEMPLATE_FILE} ${CMAKE_BINARY_DIR}/template/${TEMPLATE_FILE_NAME} COPYONLY)
endforeach()

foreach(TEMPLATE_FILE ${TEMPLATE_FILES_JINJA2})
  get_filename_component(TEMPLATE_FILE_NAME ${TEMPLATE_FILE} NAME)
  configure_file(${TEMPLATE_FILE} ${CMAKE_BINARY_DIR}/template/${TEMPLATE_FILE_NAME} COPYONLY)
endforeach()

# Copy the entire autoware_architect package directory to build directory
file(COPY ${CMAKE_SOURCE_DIR}/autoware_architect DESTINATION ${CMAKE_BINARY_DIR}/script/)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts, templates, and the autoware_architect package
install(
  DIRECTORY script/
  DESTINATION share/${PROJECT_NAME}/script
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY template/
  DESTINATION share/${PROJECT_NAME}/template
)

install(
  DIRECTORY autoware_architect/
  DESTINATION share/${PROJECT_NAME}/script/autoware_architect
)
