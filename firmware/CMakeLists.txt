# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Add nested driver components to the component path
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/components/drivers/test_utils
    ${CMAKE_CURRENT_SOURCE_DIR}/components/drivers/oled
    ${CMAKE_CURRENT_SOURCE_DIR}/components/drivers/tpic6b595
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(yarobot_control_unit)

# MANDATORY: Compile with -Werror to catch all warnings as errors
# Per tech-spec-epic-1.md: All code must compile without warnings
idf_build_set_property(COMPILE_OPTIONS "-Werror" APPEND)
