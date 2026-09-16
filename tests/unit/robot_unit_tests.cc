#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "robot/control/command_executor.h"
#include "robot/control/control_lease.h"
#include "robot/config/config_loader.h"
#include "robot/control/state_machine.h"
#include "mock_driver.h"
#include "robot/control/command_guard.h"

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    ++g_failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}

robot::CommandRequest BasicRequest(robot::CommandType type,
                                   const std::string& client_id,
                                   const std::string& lease_id,
                                   const std::string& key) {
  robot::CommandRequest request;
  request.type = type;
  request.client_id = client_id;
  request.lease_id = lease_id;
  request.idempotency_key = key;
  request.timeout = std::chrono::seconds(2);
  return request;
}

bool WaitForCommand(robot::CommandExecutor* executor, std::uint64_t command_id,
                    robot::CommandState expected,
                    std::chrono::milliseconds timeout =
                        std::chrono::milliseconds(1500)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const robot::StatusOr<robot::CommandRecord> command =
        executor->GetCommand(command_id);
    if (command.ok() && command.value().state == expected) {
      return true;
    }
    if (command.ok() &&
        (command.value().state == robot::CommandState::kFailed ||
         command.value().state == robot::CommandState::kTimedOut)) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

bool WaitForState(robot::CommandExecutor* executor,
                  robot::RobotLifecycleState expected,
                  std::chrono::milliseconds timeout =
                      std::chrono::milliseconds(1000)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (executor->GetState().lifecycle == expected) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

robot::SafetyPolicy TestSafetyPolicy() {
  robot::SafetyPolicy policy;
  policy.motion_enabled = true;
  policy.program_execution_enabled = true;
  policy.joint_limits_verified = true;
  policy.cartesian_workspace_verified = true;
  policy.maximum_joint_speed_percent = 50;
  policy.maximum_linear_speed_mm_per_second = 1000;
  policy.joint_limits.resize(6);
  for (robot::JointLimit& limit : policy.joint_limits) {
    limit.minimum_deg = -360.0;
    limit.maximum_deg = 360.0;
  }
  policy.cartesian_workspace = {-10000.0, 10000.0, -10000.0,
                                10000.0, -10000.0, 10000.0};
  policy.writable_digital_outputs = {0, 1, 2};
  policy.approved_programs = {"mock-program"};
  return policy;
}

void TestYamlConfiguration() {
  const auto config =
      robot::LoadServiceConfig("config/robots/er7_900.yaml");
  Expect(config.ok(), "ER7-900 YAML example should load");
  if (!config.ok()) {
    return;
  }
  Expect(config.value().connection.robot_id == "er7-900-01",
         "YAML robot ID should be loaded");
  Expect(config.value().connection.expected_model == "ER7-900",
         "YAML robot model should be loaded");
  Expect(!config.value().safety_policy.motion_enabled,
         "YAML example should keep motion disabled");
  const auto mock =
      robot::LoadServiceConfig("config/robots/mock.yaml");
  Expect(mock.ok() && mock.value().driver_name == "mock",
         "mock YAML example should load its driver selection");
}

void TestStateMachine() {
  robot::RobotSnapshot snapshot;
  Expect(robot::DeriveLifecycleState(snapshot) ==
             robot::RobotLifecycleState::kDisconnected,
         "disconnected state should be derived");
  snapshot.connected = true;
  snapshot.api_control = true;
  snapshot.servo_on = true;
  snapshot.controller_mode = robot::ControllerMode::kManual;
  Expect(robot::DeriveLifecycleState(snapshot) ==
             robot::RobotLifecycleState::kStandby,
         "manual mode should remain in standby for remote motion");
  snapshot.controller_mode = robot::ControllerMode::kAutomatic;
  Expect(robot::DeriveLifecycleState(snapshot) ==
             robot::RobotLifecycleState::kReady,
         "ready state should be derived");
  snapshot.moving = true;
  Expect(robot::DeriveLifecycleState(snapshot) ==
             robot::RobotLifecycleState::kExecuting,
         "executing state should be derived");
  snapshot.emergency_stop = true;
  Expect(robot::DeriveLifecycleState(snapshot) ==
             robot::RobotLifecycleState::kEmergencyStop,
         "emergency stop should have highest priority");
}

void TestControlLease() {
  robot::ControlLeaseManager leases(std::chrono::milliseconds(20));
  const auto first = leases.Acquire("client-a");
  Expect(first.ok(), "first client should acquire the lease");
  Expect(!leases.Acquire("client-b").ok(),
         "second client should not acquire an active lease");
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  Expect(!leases.IsValid(first.value().id, "client-a"),
         "expired lease should be invalid");
  Expect(leases.Acquire("client-b").ok(),
         "another client should acquire after expiry");
}

void TestCommandGuard() {
  robot::CommandGuard guard(TestSafetyPolicy());
  robot::RobotSnapshot snapshot;
  snapshot.connected = true;
  snapshot.api_control = true;
  snapshot.servo_on = true;
  snapshot.speed_ratio = 50;
  snapshot.controller_mode = robot::ControllerMode::kAutomatic;
  snapshot.lifecycle = robot::RobotLifecycleState::kReady;
  snapshot.axis_count = 6;

  robot::CommandRequest request;
  request.type = robot::CommandType::kMoveJ;
  robot::MoveJPayload payload;
  payload.target.position.degrees.resize(6);
  payload.profile.speed_percent = 10;
  request.payload = payload;
  Expect(guard.Validate(request, snapshot, true, false).ok(),
         "valid MoveJ should pass the command guard");
  std::get<robot::MoveJPayload>(request.payload)
      .target.position.degrees[0] = 500.0;
  Expect(!guard.Validate(request, snapshot, true, false).ok(),
         "out-of-range MoveJ should be rejected");

  request.type = robot::CommandType::kMoveL;
  robot::MoveLPayload linear;
  linear.profile.speed_mm_per_second = 100;
  request.payload = linear;
  Expect(guard.Validate(request, snapshot, true, false).ok(),
         "MoveL inside the verified workspace should pass");
  std::get<robot::MoveLPayload>(request.payload).target.pose.x_mm = 20000.0;
  Expect(!guard.Validate(request, snapshot, true, false).ok(),
         "MoveL outside the verified workspace should be rejected");

  request.type = robot::CommandType::kSetGlobalSpeed;
  request.payload = robot::SpeedPayload{51};
  Expect(!guard.Validate(request, snapshot, true, false).ok(),
         "global speed above the site limit should be rejected");

  request.type = robot::CommandType::kJog;
  request.payload = robot::JogCommand{};
  Expect(!guard.Validate(request, snapshot, true, false).ok(),
         "jog should not start in automatic mode");
  snapshot.controller_mode = robot::ControllerMode::kManual;
  Expect(guard.Validate(request, snapshot, true, false).ok(),
         "jog should pass its guard in manual mode");

  snapshot.controller_mode = robot::ControllerMode::kAutomatic;
  snapshot.lifecycle = robot::RobotLifecycleState::kReady;
  request.type = robot::CommandType::kLoadProgram;
  request.payload = robot::ProgramPayload{"unapproved-program"};
  Expect(!guard.Validate(request, snapshot, true, false).ok(),
         "program outside the approved list should be rejected");
  request.payload = robot::ProgramPayload{"mock-program"};
  Expect(guard.Validate(request, snapshot, true, false).ok(),
         "approved program should pass the command guard");
}

void TestCommandExecutor() {
  auto driver = std::make_unique<robot::MockDriver>(
      std::chrono::milliseconds(300));
  robot::MockDriver* const mock_driver = driver.get();
  robot::ExecutorOptions options;
  options.state_poll_interval = std::chrono::milliseconds(10);
  options.motion_start_grace = std::chrono::milliseconds(50);
  robot::CommandExecutor executor(std::move(driver), TestSafetyPolicy(),
                                  options);

  robot::ConnectionOptions connection;
  connection.robot_id = "test-robot";
  connection.address = "mock://test";
  connection.expected_model = "ER7-900";
  Expect(executor.Start(connection).ok(), "mock executor should start");

  const std::string client_id = "test-client";
  const auto lease = executor.AcquireControlLease(client_id);
  Expect(lease.ok(), "test client should acquire a lease");
  if (!lease.ok()) {
    executor.Shutdown();
    return;
  }

  auto acquire = BasicRequest(robot::CommandType::kAcquireControl, client_id,
                              lease.value().id, "acquire-control");
  const auto acquire_id = executor.Submit(acquire);
  Expect(acquire_id.ok(), "API control command should be accepted");
  Expect(acquire_id.ok() &&
             WaitForCommand(&executor, acquire_id.value(),
                            robot::CommandState::kSucceeded),
         "API control command should succeed");

  auto power = BasicRequest(robot::CommandType::kPowerOn, client_id,
                            lease.value().id, "power-on");
  const auto api_control_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!executor.GetState().api_control &&
         std::chrono::steady_clock::now() < api_control_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  Expect(executor.GetState().api_control,
         "mock state should report API control");
  const auto power_id = executor.Submit(power);
  Expect(power_id.ok(), "power-on command should be accepted");
  Expect(power_id.ok() &&
             WaitForCommand(&executor, power_id.value(),
                            robot::CommandState::kSucceeded),
         "power-on command should succeed");
  Expect(WaitForState(&executor, robot::RobotLifecycleState::kReady),
         "mock robot should become ready");

  const auto unloaded_program_id = executor.Submit(BasicRequest(
      robot::CommandType::kStartProgram, client_id, lease.value().id,
      "start-unloaded-program"));
  Expect(!unloaded_program_id.ok(),
         "starting a program not loaded by this service should be rejected");

  auto move = BasicRequest(robot::CommandType::kMoveJ, client_id,
                           lease.value().id, "move-j-1");
  robot::MoveJPayload move_payload;
  move_payload.target.position.degrees = {0.0, -10.0, 20.0,
                                          0.0, 30.0, 0.0};
  move_payload.profile.speed_percent = 10;
  move.payload = move_payload;
  const auto move_id = executor.Submit(move);
  Expect(move_id.ok(), "MoveJ command should be accepted");
  const auto retry_id = executor.Submit(move);
  Expect(retry_id.ok() && move_id.ok() && retry_id.value() == move_id.value(),
         "idempotent MoveJ retry should return the original command id");
  std::get<robot::MoveJPayload>(move.payload)
      .target.position.degrees[0] = 1.0;
  Expect(!executor.Submit(move).ok(),
         "same idempotency key with a different payload should be rejected");
  Expect(move_id.ok() &&
             WaitForCommand(&executor, move_id.value(),
                            robot::CommandState::kSucceeded),
         "MoveJ command should complete");

  auto load_program = BasicRequest(robot::CommandType::kLoadProgram,
                                   client_id, lease.value().id,
                                   "load-program");
  load_program.payload = robot::ProgramPayload{"mock-program"};
  const auto load_program_id = executor.Submit(load_program);
  Expect(load_program_id.ok() &&
             WaitForCommand(&executor, load_program_id.value(),
                            robot::CommandState::kSucceeded),
         "program load should complete");

  auto start_program = BasicRequest(robot::CommandType::kStartProgram,
                                    client_id, lease.value().id,
                                    "start-program");
  const auto start_program_id = executor.Submit(start_program);
  Expect(start_program_id.ok(), "program start should be accepted");
  Expect(WaitForState(&executor, robot::RobotLifecycleState::kExecuting),
         "running program should enter executing state");
  Expect(start_program_id.ok() &&
             WaitForCommand(&executor, start_program_id.value(),
                            robot::CommandState::kSucceeded),
         "program command should complete when the program ends");

  auto jog = BasicRequest(robot::CommandType::kJog, client_id,
                          lease.value().id, "jog-without-heartbeat");
  mock_driver->SetControllerMode(robot::ControllerMode::kManual);
  const auto manual_mode_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (executor.GetState().controller_mode !=
             robot::ControllerMode::kManual &&
         std::chrono::steady_clock::now() < manual_mode_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  robot::JogCommand jog_payload;
  jog_payload.axis = 1;
  jog_payload.start = true;
  jog.payload = jog_payload;
  const auto jog_id = executor.Submit(jog);
  Expect(jog_id.ok(), "jog command should be accepted");
  Expect(jog_id.ok() &&
             WaitForCommand(&executor, jog_id.value(),
                            robot::CommandState::kSucceeded),
         "jog start command should succeed");
  Expect(WaitForState(&executor, robot::RobotLifecycleState::kExecuting),
         "jog should enter executing state");
  Expect(WaitForState(&executor, robot::RobotLifecycleState::kStandby),
         "jog should stop after its heartbeat expires");

  mock_driver->SetControllerMode(robot::ControllerMode::kAutomatic);
  const auto automatic_mode_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (executor.GetState().controller_mode !=
             robot::ControllerMode::kAutomatic &&
         std::chrono::steady_clock::now() < automatic_mode_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  Expect(WaitForState(&executor, robot::RobotLifecycleState::kReady),
         "automatic mode should restore the ready state");

  auto lease_release_move = BasicRequest(
      robot::CommandType::kMoveJ, client_id, lease.value().id,
      "move-stopped-by-lease-release");
  lease_release_move.payload = move_payload;
  const auto lease_release_move_id = executor.Submit(lease_release_move);
  Expect(lease_release_move_id.ok(),
         "motion before lease release should be accepted");
  Expect(WaitForState(&executor, robot::RobotLifecycleState::kExecuting),
         "motion should start before releasing its lease");
  Expect(executor.ReleaseControlLease(lease.value().id, client_id).ok(),
         "lease release should succeed");
  Expect(lease_release_move_id.ok() &&
             WaitForCommand(&executor, lease_release_move_id.value(),
                            robot::CommandState::kStopped),
         "releasing the active lease should stop its motion");

  executor.Shutdown();
}

void TestShutdownCompletesOutstandingCommands() {
  auto driver = std::make_unique<robot::MockDriver>(
      std::chrono::seconds(2));
  robot::ExecutorOptions options;
  options.state_poll_interval = std::chrono::milliseconds(10);
  options.motion_start_grace = std::chrono::milliseconds(50);
  robot::CommandExecutor executor(std::move(driver), TestSafetyPolicy(),
                                  options);

  robot::ConnectionOptions connection;
  connection.robot_id = "shutdown-test-robot";
  connection.address = "mock://shutdown-test";
  connection.expected_model = "ER7-900";
  Expect(executor.Start(connection).ok(), "shutdown executor should start");

  const std::string client_id = "shutdown-test-client";
  const auto lease = executor.AcquireControlLease(client_id);
  Expect(lease.ok(), "shutdown test client should acquire a lease");
  if (!lease.ok()) {
    executor.Shutdown();
    return;
  }

  const auto acquire_id = executor.Submit(BasicRequest(
      robot::CommandType::kAcquireControl, client_id, lease.value().id,
      "shutdown-acquire-control"));
  Expect(acquire_id.ok() &&
             WaitForCommand(&executor, acquire_id.value(),
                            robot::CommandState::kSucceeded),
         "shutdown test should acquire API control");

  const auto api_control_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!executor.GetState().api_control &&
         std::chrono::steady_clock::now() < api_control_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const auto power_id = executor.Submit(BasicRequest(
      robot::CommandType::kPowerOn, client_id, lease.value().id,
      "shutdown-power-on"));
  Expect(power_id.ok() &&
             WaitForCommand(&executor, power_id.value(),
                            robot::CommandState::kSucceeded),
         "shutdown test should power on");
  Expect(WaitForState(&executor, robot::RobotLifecycleState::kReady),
         "shutdown test robot should become ready");

  auto move = BasicRequest(robot::CommandType::kMoveJ, client_id,
                           lease.value().id, "shutdown-move");
  robot::MoveJPayload move_payload;
  move_payload.target.position.degrees.resize(6);
  move_payload.profile.speed_percent = 10;
  move.payload = move_payload;
  const auto move_id = executor.Submit(move);
  Expect(move_id.ok(), "shutdown motion should be accepted");
  Expect(WaitForState(&executor, robot::RobotLifecycleState::kExecuting),
         "shutdown motion should start");

  auto set_speed = BasicRequest(robot::CommandType::kSetGlobalSpeed,
                                client_id, lease.value().id,
                                "shutdown-queued-speed");
  set_speed.payload = robot::SpeedPayload{10};
  const auto set_speed_id = executor.Submit(set_speed);
  Expect(set_speed_id.ok(), "command behind motion should be queued");

  executor.Shutdown();
  const auto stopped_motion =
      move_id.ok() ? executor.GetCommand(move_id.value())
                   : robot::StatusOr<robot::CommandRecord>(
                         robot::Status(robot::StatusCode::kNotFound,
                                       "move command was not submitted"));
  Expect(stopped_motion.ok() &&
             stopped_motion.value().state == robot::CommandState::kStopped,
         "shutdown should mark active motion as stopped");
  const auto cancelled_command =
      set_speed_id.ok()
          ? executor.GetCommand(set_speed_id.value())
          : robot::StatusOr<robot::CommandRecord>(
                robot::Status(robot::StatusCode::kNotFound,
                              "speed command was not submitted"));
  Expect(cancelled_command.ok() &&
             cancelled_command.value().state ==
                 robot::CommandState::kCancelled,
         "shutdown should cancel queued commands");
}

}  // namespace

int main() {
  TestYamlConfiguration();
  TestStateMachine();
  TestControlLease();
  TestCommandGuard();
  TestCommandExecutor();
  TestShutdownCompletesOutstandingCommands();
  if (g_failures == 0) {
    std::cout << "All robot-manager tests passed\n";
    return EXIT_SUCCESS;
  }
  std::cerr << g_failures << " test(s) failed\n";
  return EXIT_FAILURE;
}
