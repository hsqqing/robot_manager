#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "mock_driver.h"
#include "robot/types/command.h"
#include "robot/control/command_executor.h"

namespace {

robot::SafetyPolicy ExamplePolicy() {
  robot::SafetyPolicy policy;
  policy.motion_enabled = true;
  policy.joint_limits_verified = true;
  policy.maximum_joint_speed_percent = 20;
  policy.joint_limits.resize(6);
  for (auto& limit : policy.joint_limits) {
    limit.minimum_deg = -360.0;
    limit.maximum_deg = 360.0;
  }
  return policy;
}

bool WaitFor(robot::CommandExecutor& executor, std::uint64_t id) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto record = executor.GetCommand(id);
    if (record.ok() && record.value().state == robot::CommandState::kSucceeded) {
      return true;
    }
    if (record.ok() && record.value().state != robot::CommandState::kQueued &&
        record.value().state != robot::CommandState::kRunning) {
      std::cerr << "command did not succeed: "
                << record.value().result.message() << '\n';
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

}  // namespace

int main() {
  robot::ExecutorOptions options;
  options.state_poll_interval = std::chrono::milliseconds(10);
  options.motion_start_grace = std::chrono::milliseconds(50);
  robot::CommandExecutor executor(std::make_unique<robot::MockDriver>(),
                                  ExamplePolicy(), options);

  robot::ConnectionOptions connection;
  connection.robot_id = "example-robot";
  connection.address = "mock://example";
  const auto start = executor.Start(connection);
  if (!start.ok()) {
    std::cerr << "start: " << start.message() << '\n';
    return 1;
  }

  const auto lease = executor.AcquireControlLease("basic-move-example");
  if (!lease.ok()) {
    std::cerr << "lease: " << lease.status().message() << '\n';
    return 1;
  }
  auto submit = [&](robot::CommandType type, robot::CommandPayload payload,
                    const char* key) {
    robot::CommandRequest request;
    request.type = type;
    request.payload = std::move(payload);
    request.client_id = "basic-move-example";
    request.lease_id = lease.value().id;
    request.idempotency_key = key;
    return executor.Submit(std::move(request));
  };

  const auto acquire = submit(robot::CommandType::kAcquireControl,
                              std::monostate{}, "acquire-control");
  if (!acquire.ok()) {
    std::cerr << "submit acquire: " << acquire.status().message() << '\n';
    executor.Shutdown();
    return 1;
  }
  if (!WaitFor(executor, acquire.value())) {
    std::cerr << "acquire command failed\n";
    executor.Shutdown();
    return 1;
  }
  const auto power = submit(robot::CommandType::kPowerOn, std::monostate{},
                            "power-on");
  if (!power.ok() || !WaitFor(executor, power.value())) {
    executor.Shutdown();
    return 1;
  }

  robot::MoveJPayload move;
  move.target.position.degrees = {0.0, -10.0, 20.0, 0.0, 30.0, 0.0};
  move.profile.speed_percent = 10;
  const auto command = submit(robot::CommandType::kMoveJ, move, "move-j-1");
  const bool succeeded = command.ok() && WaitFor(executor, command.value());
  executor.Shutdown();
  return succeeded ? 0 : 1;
}
