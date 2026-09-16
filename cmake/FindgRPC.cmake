# Prefer the official gRPC CMake package when the installed distribution
# provides it. Ubuntu 20.04's libgrpc++-dev package does not, so fall back to
# the headers, shared library, and code generator installed by that package.
find_package(gRPC CONFIG QUIET)

if(NOT TARGET gRPC::grpc++)
  find_path(GRPC_INCLUDE_DIR NAMES grpcpp/grpcpp.h)
  find_library(GRPC_CPP_LIBRARY NAMES grpc++)
  find_program(GRPC_CPP_PLUGIN_EXECUTABLE NAMES grpc_cpp_plugin)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(
    gRPC
    REQUIRED_VARS
      GRPC_INCLUDE_DIR
      GRPC_CPP_LIBRARY
      GRPC_CPP_PLUGIN_EXECUTABLE)

  if(gRPC_FOUND)
    add_library(gRPC::grpc++ SHARED IMPORTED)
    set_target_properties(
      gRPC::grpc++
      PROPERTIES
        IMPORTED_LOCATION "${GRPC_CPP_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${GRPC_INCLUDE_DIR}")

    add_executable(gRPC::grpc_cpp_plugin IMPORTED)
    set_target_properties(
      gRPC::grpc_cpp_plugin
      PROPERTIES IMPORTED_LOCATION "${GRPC_CPP_PLUGIN_EXECUTABLE}")
  endif()
endif()

if(TARGET gRPC::grpc++)
  set(gRPC_FOUND TRUE)
endif()
