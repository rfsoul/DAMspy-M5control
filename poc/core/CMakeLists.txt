# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "../hid_tunnel_protocol")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

if(NOT DEFINED CORE_FIRMWARE_VERSION)
    set(CORE_FIRMWARE_VERSION "ota-poc-1")
endif()
set(PROJECT_VER "${CORE_FIRMWARE_VERSION}")
project(hello_world)
