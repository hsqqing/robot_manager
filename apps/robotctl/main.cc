#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "robot/version.h"

#ifdef ROBOT_HAS_GRPC
#include <grpcpp/grpcpp.h>

#include "robot/v1/robot_control.grpc.pb.h"
#endif

namespace {

void PrintUsage() {
  std::cout << "Usage:\n"
            << "  robotctl version\n"
            << "  robotctl models\n"
            << "  robotctl state <endpoint> <robot-id>\n";
}

int PrintVersion() {
  std::cout << "robotctl " << ROBOT_VERSION_STRING << '\n'
            << "git: " << ROBOT_GIT_COMMIT_HASH << " ("
            << ROBOT_GIT_BRANCH << ")\n"
            << "built: " << ROBOT_BUILD_DATE << ' ' << ROBOT_BUILD_TIME
            << " with " << ROBOT_COMPILER_INFO << '\n';
  return 0;
}

#ifdef ROBOT_HAS_GRPC
int PrintState(const std::string& endpoint, const std::string& robot_id) {
  std::shared_ptr<grpc::Channel> channel = grpc::CreateChannel(
      endpoint, grpc::InsecureChannelCredentials());
  std::unique_ptr<robot::v1::RobotControlService::Stub> stub =
      robot::v1::RobotControlService::NewStub(channel);
  robot::v1::GetStateRequest request;
  request.set_robot_id(robot_id);
  robot::v1::RobotState response;
  grpc::ClientContext context;
  const grpc::Status status = stub->GetState(&context, request, &response);
  if (!status.ok()) {
    std::cerr << "GetState failed: " << status.error_message() << '\n';
    return 1;
  }
  std::cout << "robot_id=" << response.robot_id()
            << " model=" << response.controller_model()
            << " state=" << response.lifecycle()
            << " connected=" << response.connected()
            << " servo=" << response.servo_on()
            << " moving=" << response.moving() << '\n';
  return 0;
}
#endif

}  // namespace

int main(int argc, char* argv[]) {
  if (argc == 2 && std::string(argv[1]) == "version") {
    return PrintVersion();
  }
  if (argc == 4 && std::string(argv[1]) == "state") {
#ifdef ROBOT_HAS_GRPC
    return PrintState(argv[2], argv[3]);
#else
    std::cerr << "robotctl was built without gRPC support\n";
    return 2;
#endif
  }
  PrintUsage();
  return 2;
}
