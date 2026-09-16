find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)

set(ROBOT_PROTO_ROOT "${PROJECT_SOURCE_DIR}/api/proto")
set(ROBOT_PROTO_FILE
    "${ROBOT_PROTO_ROOT}/robot/v1/robot_control.proto")
set(ROBOT_PROTO_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${ROBOT_PROTO_GENERATED_DIR}")

add_custom_command(
  OUTPUT
    "${ROBOT_PROTO_GENERATED_DIR}/robot/v1/robot_control.pb.cc"
    "${ROBOT_PROTO_GENERATED_DIR}/robot/v1/robot_control.pb.h"
    "${ROBOT_PROTO_GENERATED_DIR}/robot/v1/robot_control.grpc.pb.cc"
    "${ROBOT_PROTO_GENERATED_DIR}/robot/v1/robot_control.grpc.pb.h"
  COMMAND
    protobuf::protoc
    --proto_path=${ROBOT_PROTO_ROOT}
    --cpp_out=${ROBOT_PROTO_GENERATED_DIR}
    --grpc_out=${ROBOT_PROTO_GENERATED_DIR}
    --plugin=protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>
    ${ROBOT_PROTO_FILE}
  DEPENDS "${ROBOT_PROTO_FILE}"
  VERBATIM)

add_library(
  robot_proto
  "${ROBOT_PROTO_GENERATED_DIR}/robot/v1/robot_control.pb.cc"
  "${ROBOT_PROTO_GENERATED_DIR}/robot/v1/robot_control.grpc.pb.cc")
target_include_directories(robot_proto PUBLIC "${ROBOT_PROTO_GENERATED_DIR}")
target_link_libraries(robot_proto PUBLIC protobuf::libprotobuf gRPC::grpc++)

add_library(robot_grpc grpc_server.cc)
target_include_directories(
  robot_grpc
  PUBLIC "${PROJECT_SOURCE_DIR}/include"
         "${PROJECT_SOURCE_DIR}/apps")
target_link_libraries(robot_grpc PUBLIC robot_core robot_proto gRPC::grpc++)
target_compile_definitions(robot_grpc PUBLIC ROBOT_HAS_GRPC=1)
robot_enable_warnings(robot_grpc)
