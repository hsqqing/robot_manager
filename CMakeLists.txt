cmake_minimum_required(VERSION 3.22)

project(robot_manager VERSION 0.1.0 LANGUAGES CXX)

set(ROBOT_VERSION_STAGE "release" CACHE STRING
    "Version stage: dev, beta, rc, or release")
set_property(CACHE ROBOT_VERSION_STAGE PROPERTY STRINGS dev beta rc release)

find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${CMAKE_SOURCE_DIR}/.git")
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" rev-parse --short HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE ROBOT_GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE ROBOT_GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
endif()
if(NOT ROBOT_GIT_COMMIT_HASH)
  set(ROBOT_GIT_COMMIT_HASH "unknown")
endif()
if(NOT ROBOT_GIT_BRANCH OR ROBOT_GIT_BRANCH STREQUAL "HEAD")
  set(ROBOT_GIT_BRANCH "unknown")
endif()

string(TIMESTAMP ROBOT_BUILD_DATE "%Y-%m-%d")
string(TIMESTAMP ROBOT_BUILD_TIME "%H:%M:%S")
set(ROBOT_COMPILER_INFO
    "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
set(ROBOT_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${ROBOT_GENERATED_INCLUDE_DIR}/robot")
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/version.h.in"
  "${ROBOT_GENERATED_INCLUDE_DIR}/robot/version.h"
  @ONLY)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "robot-manager is supported only on Linux")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ROBOT_ENABLE_EFORT "Build the Linux EFORT SDK adapter" OFF)
option(ROBOT_ENABLE_MOCK "Build the in-memory mock adapter" ON)
option(ROBOT_ENABLE_GRPC "Build the gRPC service transport" OFF)
option(ROBOT_BUILD_APPS "Build the example service and command-line client" ON)
option(ROBOT_BUILD_EXAMPLES "Build integration examples" OFF)
option(ROBOT_BUILD_TESTS "Build robot-manager tests" ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CompilerWarnings)
include(GNUInstallDirs)
find_package(Threads REQUIRED)

add_library(
  robot_core
  src/control/state_machine.cc
  src/control/control_lease.cc
  src/control/command_executor.cc
  src/control/command_guard.cc)

target_include_directories(
  robot_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_link_libraries(robot_core PUBLIC Threads::Threads)
robot_enable_warnings(robot_core)

# Deployment configuration belongs to the executable composition, not the framework.
if(ROBOT_BUILD_APPS OR ROBOT_BUILD_TESTS)
  add_library(robot_config src/config/config_loader.cc)
  target_link_libraries(robot_config PUBLIC robot_core)
  target_include_directories(robot_config SYSTEM PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/rapidyaml/include")
  robot_enable_warnings(robot_config)
endif()

# Optional in-memory implementation.  Keeping it outside robot_core means
# applications that provide their own driver do not pull test/dev code in.
if(ROBOT_ENABLE_MOCK)
  add_subdirectory(src/adapters/mock)
endif()
if(ROBOT_ENABLE_EFORT)
  add_subdirectory(src/adapters/efort)
endif()
# The generated protobuf/gRPC library is the compiled form of the api/proto
# contract, so it is defined in api/ where every app links it, instead of
# inside a single app's transport directory.
if(ROBOT_ENABLE_GRPC)
  add_subdirectory(api)
endif()
if(ROBOT_BUILD_APPS)
  add_subdirectory(apps)
endif()
if(ROBOT_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(ROBOT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
