# The generated protobuf/gRPC library is the compiled form of the api/proto
# contract, so it is defined here where every app links it, instead of inside
# a single app's transport directory.
find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)

set(ROBOT_PROTO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/proto")
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
