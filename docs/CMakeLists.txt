# Standalone so CI can render the site with only Doxygen installed:
#   cmake -S docs -B build/docs && cmake --build build/docs

cmake_minimum_required(VERSION 3.28)
project(vive_vr_ros2_docs NONE)

# Below 1.16, doxygen's own div.fragment rule outranks the theme's .fragment and the pages
# render subtly wrong rather than failing.
find_package(Doxygen 1.16 REQUIRED)

get_filename_component(VIVE_VR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

# Read from package.xml rather than duplicated here: standalone there is no project() to ask.
file(READ "${VIVE_VR_ROOT}/package.xml" _package_xml)
string(REGEX MATCH "<version>([^<]*)</version>" _ "${_package_xml}")
set(VIVE_VR_VERSION "${CMAKE_MATCH_1}")

set(_doxyfile "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")
configure_file(doxygen/Doxyfile.in "${_doxyfile}" @ONLY)

add_custom_target(docs ALL
  COMMAND ${DOXYGEN_EXECUTABLE} "${_doxyfile}"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  COMMENT "Generating documentation with Doxygen ${DOXYGEN_VERSION}"
  VERBATIM
)
