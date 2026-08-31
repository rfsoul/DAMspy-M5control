cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "../hid_tunnel_protocol")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

if(NOT DEFINED STICK_FIRMWARE_VERSION)
    set(STICK_FIRMWARE_VERSION "gateway-poc-1")
endif()
set(PROJECT_VER "${STICK_FIRMWARE_VERSION}")
project(espnow_echo_test)
