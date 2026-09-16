#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "robot/service/command_executor.h"
#include "robot/config/config_loader.h"
#include "robot/domain/state_machine.h"
#ifdef ROBOT_HAS_MOCK
#include "mock_driver.h"
#endif
#include "robot/version.h"

#ifdef ROBOT_HAS_EFORT
#include "efort_driver.h"
#endif

#ifdef ROBOT_HAS_GRPC
#include "robot_manager/transport/grpc_server.h"
#endif

namespace {

constexpr char kDefaultConfigPath[] = "/etc/robot-manager/robot.yaml";
volatile std::sig_atomic_t g_stop_requested = 0;

void HandleSignal(int) { g_stop_requested = 1; }

void PrintUsage(const char* program) {
  std::cerr << "Usage: " << program << " [--config <path>] [--version]\n";
}

void PrintVersion() {
  std::cout << "robot-manager " << ROBOT_VERSION_STRING << '\n'
            << "git: " << ROBOT_GIT_COMMIT_HASH << " (" << ROBOT_GIT_BRANCH
            << ")\n"
            << "built: " << ROBOT_BUILD_DATE << ' ' << ROBOT_BUILD_TIME
            << " with " << ROBOT_COMPILER_INFO << '\n';
}

std::unique_ptr<robot::IRobotDriver> CreateDriver(
    const std::string& driver_name) {
#ifdef ROBOT_HAS_MOCK
  if (driver_name == "mock") {
    return std::make_unique<robot::MockDriver>();
  }
#endif
#ifdef ROBOT_HAS_EFORT
  if (driver_name == "efort") {
    return std::make_unique<robot::EfortDriver>();
  }
#endif
  return nullptr;
}

int Run(const std::string& config_path) {
  // 主进程负责组装依赖：配置 -> IRobotDriver -> CommandExecutor -> gRPC。
  // 业务规则位于应用/领域层，main 不参与具体运动逻辑。
  const robot::StatusOr<robot::ServiceConfig> config =
      robot::LoadServiceConfig(config_path);
  if (!config.ok()) {
    std::cerr << config.status().message() << '\n';
    return 2;
  }

  std::unique_ptr<robot::IRobotDriver> driver =
      CreateDriver(config.value().driver_name);
  if (driver == nullptr) {
    std::cerr << "driver '" << config.value().driver_name
              << "' is unavailable in this build\n";
    return 2;
  }

  robot::CommandExecutor executor(std::move(driver),
                                  config.value().safety_policy,
                                  config.value().executor_options);
  const robot::Status start_status = executor.Start(config.value().connection);
  if (!start_status.ok()) {
    std::cerr << "failed to start robot service: " << start_status.message()
              << " (vendor_code=" << start_status.vendor_code() << ")\n";
    return 1;
  }

  std::cout << "robot-manager instance=" << config.value().instance_id
            << " connected to "
            << config.value().connection.expected_model
            << " at " << config.value().connection.address
            << ", robot_id=" << config.value().connection.robot_id << '\n';

#ifdef ROBOT_HAS_GRPC
  // RunGrpcServer 会阻塞到收到 SIGINT/SIGTERM；返回后统一关闭执行器。
  const robot::Status server_status =
      robot::RunGrpcServer(config.value().grpc, config.value().connection.robot_id,
                           &executor, &g_stop_requested);
  if (!server_status.ok()) {
    std::cerr << server_status.message() << '\n';
    executor.Shutdown();
    return 1;
  }
#else
  std::cout << "gRPC transport is disabled in this build; state monitor only\n";
  while (g_stop_requested == 0) {
    const robot::RobotSnapshot state = executor.GetState();
    std::cout << "state=" << robot::LifecycleStateName(state.lifecycle)
              << " connected=" << state.connected << " servo=" << state.servo_on
              << " moving=" << state.moving << '\n';
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
#endif

  executor.Shutdown();
  return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
  std::string config_path = kDefaultConfigPath;
  if (argc == 2 && std::string(argv[1]) == "--version") {
    PrintVersion();
    return 0;
  }
  if (argc == 3 && std::string(argv[1]) == "--config") {
    config_path = argv[2];
  } else if (argc != 1) {
    PrintUsage(argv[0]);
    return 2;
  }
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);
  return Run(config_path);
}
