if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "The bundled EFORT SDK target is Linux only")
endif()

if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  message(FATAL_ERROR
    "libEftSdk.so is x86_64. Current target: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(EFORT_SDK_ROOT "${CMAKE_CURRENT_LIST_DIR}/../3rdparty/efort_sdk"
    CACHE PATH "Root of the EFORT C++ SDK")

find_path(
  EFORT_SDK_INCLUDE_DIR
  NAMES EfortSdk.h
  PATHS "${EFORT_SDK_ROOT}/include"
  NO_DEFAULT_PATH)

find_library(
  EFORT_SDK_LIBRARY
  NAMES EftSdk
  PATHS "${EFORT_SDK_ROOT}/lib"
  NO_DEFAULT_PATH)

find_library(
  EFORT_LOG4CPP_LIBRARY
  NAMES log4cpp
  PATHS "${EFORT_SDK_ROOT}/lib"
  NO_DEFAULT_PATH)

find_file(
  EFORT_RLIBCPP_BCC_LIBRARY
  NAMES librlibcpp.bcc.so.1
  PATHS "${EFORT_SDK_ROOT}/lib"
  NO_DEFAULT_PATH)

find_file(
  EFORT_RLIBCPP_TOOL_LIBRARY
  NAMES librlibcpp.tool.so.1
  PATHS "${EFORT_SDK_ROOT}/lib"
  NO_DEFAULT_PATH)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  EfortSdk
  REQUIRED_VARS
    EFORT_SDK_INCLUDE_DIR
    EFORT_SDK_LIBRARY
    EFORT_LOG4CPP_LIBRARY
    EFORT_RLIBCPP_BCC_LIBRARY
    EFORT_RLIBCPP_TOOL_LIBRARY)

if(EfortSdk_FOUND AND NOT TARGET EfortSdk::EfortSdk)
  set(EFORT_RUNTIME_LIBRARIES
      "${EFORT_LOG4CPP_LIBRARY}"
      "${EFORT_RLIBCPP_BCC_LIBRARY}"
      "${EFORT_RLIBCPP_TOOL_LIBRARY}")
  add_library(EfortSdk::EfortSdk SHARED IMPORTED)
  set_target_properties(
    EfortSdk::EfortSdk
    PROPERTIES
      IMPORTED_LOCATION "${EFORT_SDK_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${EFORT_SDK_INCLUDE_DIR}"
      INTERFACE_LINK_LIBRARIES "${EFORT_RUNTIME_LIBRARIES}")
endif()
