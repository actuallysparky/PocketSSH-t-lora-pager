# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Project version
set(PROJECT_VER "1.2.0")

# TPAGER_TARGET contract:
# - Propagate only the libssh2 packet-size override globally so managed
#   components pick it up.
# - Do not publish TPAGER_TARGET to every component to avoid accidental macro
#   collisions outside the app code.
if(TPAGER_TARGET)
    add_compile_definitions(LIBSSH2_MAX_SSH_PACKET_LEN=8000)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(PocketSSH)
