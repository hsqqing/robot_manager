# robot_proto is defined in api/CMakeLists.txt, next to the contract it
# compiles; this directory only owns the gRPC transport that serves it.
add_library(robot_grpc grpc_server.cc)
target_include_directories(
  robot_grpc
  PUBLIC "${PROJECT_SOURCE_DIR}/include"
         "${PROJECT_SOURCE_DIR}/apps")
target_link_libraries(robot_grpc PUBLIC robot_core robot_proto gRPC::grpc++)
target_compile_definitions(robot_grpc PUBLIC ROBOT_HAS_GRPC=1)
robot_enable_warnings(robot_grpc)
