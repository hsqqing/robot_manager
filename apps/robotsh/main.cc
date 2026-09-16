// 独立框架 CLI：直接组装驱动和 CommandExecutor，不依赖 robot-manager 守护进程
// 或任何传输协议。用于在没有 gRPC 服务的环境下手动驱动、验证和诊断。
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "robot/config/config_loader.h"
#include "robot/control/command_executor.h"
#include "robot/control/state_machine.h"
#include "robot/version.h"
#ifdef ROBOT_HAS_MOCK
#include "mock_driver.h"
#endif
#ifdef ROBOT_HAS_EFORT
#include "efort_driver.h"
#endif

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

void HandleSignal(int) { g_stop_requested = 1; }

void PrintUsage(const char* program) {
  std::cerr
      << "Usage:\n"
      << "  " << program << " version\n"
      << "  " << program << " state <config-path>\n"
      << "  " << program
      << " movej <config-path> <j1> <j2> <j3> <j4> <j5> <j6> [speed-percent]\n"
      << "\n"
      << "Uses the framework directly (driver + CommandExecutor); it does\n"
      << "not contact a running robot-manager and does not use gRPC.\n";
}

void PrintVersion() {
  std::cout << "robotsh " << ROBOT_VERSION_STRING << '\n'
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

int PrintState(const std::string& config_path) {
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
  const robot::Status start_status =
      executor.Start(config.value().connection);
  if (!start_status.ok()) {
    std::cerr << "failed to start robot service: " << start_status.message()
              << " (vendor_code=" << start_status.vendor_code() << ")\n";
    return 1;
  }
  const robot::RobotSnapshot state = executor.GetState();
  std::cout << "robot_id=" << config.value().connection.robot_id
            << " model=" << state.controller_model
            << " state=" << robot::LifecycleStateName(state.lifecycle)
            << " connected=" << state.connected
            << " servo=" << state.servo_on << " moving=" << state.moving
            << " alarm=" << state.alarm_active << '\n';
  executor.Shutdown();
  return 0;
}

bool WaitSnapshot(robot::CommandExecutor& executor,
                  bool (*predicate)(const robot::RobotSnapshot&),
                  const char* description) {
  // 命令校验基于轮询快照，提交下一条命令前先等状态可观测，
  // 与单元测试和集成指南要求的客户端行为一致。
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < deadline) {
    if (g_stop_requested != 0) {
      return false;
    }
    if (predicate(executor.GetState())) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::cerr << "timeout waiting for " << description << '\n';
  return false;
}

bool WaitDone(robot::CommandExecutor& executor, std::uint64_t command_id) {
  while (g_stop_requested == 0) {
    const auto record = executor.GetCommand(command_id);
    if (!record.ok()) {
      std::cerr << "get command: " << record.status().message() << '\n';
      return false;
    }
    switch (record.value().state) {
      case robot::CommandState::kSucceeded:
        return true;
      case robot::CommandState::kFailed:
      case robot::CommandState::kCancelled:
      case robot::CommandState::kTimedOut:
      case robot::CommandState::kStopped:
        std::cerr << "command did not succeed: "
                  << record.value().result.message() << '\n';
        return false;
      case robot::CommandState::kQueued:
      case robot::CommandState::kRunning:
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::cerr << "interrupted; submitting stop\n";
  return false;
}

int RunMoveJ(const std::string& config_path, int argc, char* argv[]) {
  robot::MoveJPayload payload;
  payload.target.position.degrees.resize(6);
  for (int i = 0; i < 6; ++i) {
    payload.target.position.degrees[static_cast<std::size_t>(i)] =
        std::strtod(argv[3 + i], nullptr);
  }
  if (argc >= 10) {
    const long speed = std::strtol(argv[9], nullptr, 10);
    if (speed <= 0 || speed > 100) {
      std::cerr << "speed-percent must be in (0, 100]\n";
      return 2;
    }
    payload.profile.speed_percent = static_cast<int>(speed);
  }

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
  const robot::Status start_status =
      executor.Start(config.value().connection);
  if (!start_status.ok()) {
    std::cerr << "failed to start robot service: " << start_status.message()
              << " (vendor_code=" << start_status.vendor_code() << ")\n";
    return 1;
  }
  std::cout << "connected to " << config.value().connection.expected_model
            << " at " << config.value().connection.address << '\n';

  const auto lease = executor.AcquireControlLease("robotsh");
  if (!lease.ok()) {
    std::cerr << "lease: " << lease.status().message() << '\n';
    executor.Shutdown();
    return 1;
  }
  auto submit = [&](robot::CommandType type, robot::CommandPayload body,
                    const char* key) {
    robot::CommandRequest request;
    request.type = type;
    request.payload = std::move(body);
    request.client_id = "robotsh";
    request.lease_id = lease.value().id;
    request.idempotency_key = key;
    return executor.Submit(std::move(request));
  };

  bool ok = true;
  const auto acquire =
      submit(robot::CommandType::kAcquireControl, std::monostate{}, "acquire");
  if (!acquire.ok()) {
    std::cerr << "submit acquire: " << acquire.status().message() << '\n';
    ok = false;
  } else if (WaitDone(executor, acquire.value()) &&
             WaitSnapshot(executor,
                          [](const robot::RobotSnapshot& snapshot) {
                            return snapshot.api_control;
                          },
                          "API control")) {
    const auto power =
        submit(robot::CommandType::kPowerOn, std::monostate{}, "power-on");
    if (!power.ok()) {
      std::cerr << "submit power-on: " << power.status().message() << '\n';
      ok = false;
    } else if (WaitDone(executor, power.value()) &&
               WaitSnapshot(executor,
                            [](const robot::RobotSnapshot& snapshot) {
                              return snapshot.lifecycle ==
                                     robot::RobotLifecycleState::kReady;
                            },
                            "servo-on (READY state)")) {
      const auto move = submit(robot::CommandType::kMoveJ, payload, "movej");
      if (!move.ok()) {
        std::cerr << "submit movej: " << move.status().message() << '\n';
        ok = false;
      } else {
        ok = WaitDone(executor, move.value());
      }
    } else {
      ok = false;
    }
  } else {
    ok = false;
  }
  executor.Shutdown();
  return ok ? 0 : 1;
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc == 2 && std::string(argv[1]) == "version") {
    PrintVersion();
    return 0;
  }
  if (argc == 3 && std::string(argv[1]) == "state") {
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);
    return PrintState(argv[2]);
  }
  if (argc >= 9 && std::string(argv[1]) == "movej") {
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);
    return RunMoveJ(argv[2], argc, argv);
  }
  PrintUsage(argv[0]);
  return 2;
}
