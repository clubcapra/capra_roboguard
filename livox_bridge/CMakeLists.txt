cmake_minimum_required(VERSION 3.10)
project(livox_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Livox-SDK2 is built in-tree (not installed system-wide). Point this at the
# clone+build; override with -DLIVOX_SDK2_DIR=... if it lives elsewhere.
set(LIVOX_SDK2_DIR "/home/capra/Livox-SDK2" CACHE PATH "Livox-SDK2 source + build dir")

set(LIVOX_SDK2_LIB "${LIVOX_SDK2_DIR}/build/sdk_core/liblivox_lidar_sdk_static.a")
if(NOT EXISTS "${LIVOX_SDK2_LIB}")
  message(FATAL_ERROR "Livox-SDK2 static lib not found at ${LIVOX_SDK2_LIB}. "
                      "Build Livox-SDK2 first, or set -DLIVOX_SDK2_DIR.")
endif()

add_executable(livox_bridge src/livox_bridge.cpp)
target_include_directories(livox_bridge PRIVATE ${LIVOX_SDK2_DIR}/include)
target_link_libraries(livox_bridge ${LIVOX_SDK2_LIB} pthread)
