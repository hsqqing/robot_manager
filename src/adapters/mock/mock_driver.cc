// 用于测试和本地演示的确定性机器人驱动实现。
#include "mock_driver.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "robot/domain/state_machine.h"

namespace robot {

namespace {
constexpr std::uint32_t kMockDigitalIoCount = 176;
}

MockDriver::MockDriver(std::chrono::milliseconds motion_duration)
    : motion_duration_(motion_duration),
      digital_inputs_(kMockDigitalIoCount, false),
      digital_outputs_(kMockDigitalIoCount, false) {}

Status MockDriver::Connect(const ConnectionOptions& options) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_.connected) {
    return Status(StatusCode::kAlreadyExists,
                  "mock robot is already connected");
  }
  if (options.address.empty()) {
    return Status(StatusCode::kInvalidArgument, "robot address is required");
  }
  const std::size_t axis_count =
      options.expected_axis_count == 0 ? 6 : options.expected_axis_count;
  state_.connected = true;
  state_.controller_model = options.expected_model.empty() ? "mock" : options.expected_model;
  state_.axis_count = axis_count;
  state_.joints.degrees.resize(axis_count);
  state_.controller_mode = ControllerMode::kAutomatic;
  state_.speed_ratio = 10;
  state_.active_tool = "tool0";
  state_.active_work_object = "wobj0";
  state_.observed_at = std::chrono::steady_clock::now();
  state_.lifecycle = DeriveLifecycleState(state_);
  return Status::Ok();
}

Status MockDriver::Disconnect() {
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = RobotSnapshot{};
  program_loaded_ = false;
  program_running_ = false;
  jog_active_ = false;
  return Status::Ok();
}

Status MockDriver::AcquireControl() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!state_.connected) {
    return Status(StatusCode::kFailedPrecondition,
                  "mock robot is not connected");
  }
  state_.api_control = true;
  return Status::Ok();
}

Status MockDriver::ReleaseControl() {
  std::lock_guard<std::mutex> lock(mutex_);
  state_.api_control = false;
  return Status::Ok();
}

Status MockDriver::PowerOn() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!state_.connected || !state_.api_control || state_.emergency_stop ||
      state_.alarm_active) {
    return Status(StatusCode::kFailedPrecondition,
                  "mock power-on preconditions are not satisfied");
  }
  state_.servo_on = true;
  return Status::Ok();
}

Status MockDriver::PowerOff() {
  std::lock_guard<std::mutex> lock(mutex_);
  state_.moving = false;
  state_.paused = false;
  state_.servo_on = false;
  state_.program_running = false;
  program_running_ = false;
  jog_active_ = false;
  return Status::Ok();
}

Status MockDriver::ClearFault() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_.emergency_stop) {
    return Status(StatusCode::kFailedPrecondition,
                  "emergency stop is still active");
  }
  state_.alarm_active = false;
  return Status::Ok();
}

Status MockDriver::SetGlobalSpeed(unsigned int ratio) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (ratio < 1 || ratio > 100) {
    return Status(StatusCode::kInvalidArgument,
                  "speed ratio must be in [1, 100]");
  }
  state_.speed_ratio = ratio;
  return Status::Ok();
}

Status MockDriver::MoveJ(const JointTarget& target,
                              const JointMotionProfile& profile) {
  std::lock_guard<std::mutex> lock(mutex_);
  const Status ready = CheckReadyLocked();
  if (!ready.ok()) {
    return ready;
  }
  if (profile.speed_percent < 1 || profile.speed_percent > 100) {
    return Status(StatusCode::kInvalidArgument,
                  "joint speed must be in [1, 100]");
  }
  state_.joints = target.position;
  state_.active_tool = target.tool_name;
  state_.active_work_object = target.work_object_name;
  BeginMotionLocked();
  return Status::Ok();
}

Status MockDriver::MoveL(const CartesianTarget& target,
                              const CartesianMotionProfile& profile) {
  std::lock_guard<std::mutex> lock(mutex_);
  const Status ready = CheckReadyLocked();
  if (!ready.ok()) {
    return ready;
  }
  if (profile.speed_mm_per_second < 1) {
    return Status(StatusCode::kInvalidArgument,
                  "linear speed must be positive");
  }
  state_.tcp_pose = target.pose;
  state_.active_tool = target.tool_name;
  state_.active_work_object = target.work_object_name;
  BeginMotionLocked();
  return Status::Ok();
}

Status MockDriver::MoveC(const CircularTarget& target,
                              const CartesianMotionProfile& profile) {
  std::lock_guard<std::mutex> lock(mutex_);
  const Status ready = CheckReadyLocked();
  if (!ready.ok()) {
    return ready;
  }
  if (profile.speed_mm_per_second < 1) {
    return Status(StatusCode::kInvalidArgument,
                  "circular speed must be positive");
  }
  state_.tcp_pose = target.target;
  state_.active_tool = target.tool_name;
  state_.active_work_object = target.work_object_name;
  BeginMotionLocked();
  return Status::Ok();
}

Status MockDriver::Hold() {
  std::lock_guard<std::mutex> lock(mutex_);
  RefreshMotionLocked();
  if (!state_.moving) {
    return Status(StatusCode::kFailedPrecondition, "mock robot is not moving");
  }
  state_.moving = false;
  state_.paused = true;
  return Status::Ok();
}

Status MockDriver::Resume() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!state_.paused) {
    return Status(StatusCode::kFailedPrecondition, "mock robot is not paused");
  }
  state_.paused = false;
  BeginMotionLocked();
  return Status::Ok();
}

Status MockDriver::Stop(StopMode mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode == StopMode::kQuick) {
    return Status(StatusCode::kFailedPrecondition,
                  "EFORT SDK V2.8 has no distinct quick-stop API");
  }
  state_.moving = false;
  state_.paused = false;
  state_.program_running = false;
  program_running_ = false;
  jog_active_ = false;
  if (mode == StopMode::kPowerOffRequest) {
    state_.servo_on = false;
  }
  return Status::Ok();
}

Status MockDriver::Jog(const JogCommand& command) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (command.axis < 1 || command.axis > state_.axis_count) {
    return Status(StatusCode::kInvalidArgument,
                  "jog axis exceeds the mock robot axis count");
  }
  if (!command.start) {
    state_.moving = false;
    state_.paused = false;
    jog_active_ = false;
    return Status::Ok();
  }
  if (jog_active_) {
    return Status::Ok();
  }
  const Status ready = CheckReadyLocked();
  if (!ready.ok()) {
    return ready;
  }
  state_.moving = true;
  state_.paused = false;
  jog_active_ = true;
  return Status::Ok();
}

Status MockDriver::LoadProgram(const std::string& name) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!state_.connected || !state_.api_control) {
    return Status(StatusCode::kFailedPrecondition,
                  "mock robot is not under API control");
  }
  if (name.empty()) {
    return Status(StatusCode::kInvalidArgument, "program name is required");
  }
  program_loaded_ = true;
  return Status::Ok();
}

Status MockDriver::StartProgram() {
  std::lock_guard<std::mutex> lock(mutex_);
  const Status ready = CheckReadyLocked();
  if (!ready.ok()) {
    return ready;
  }
  if (!program_loaded_) {
    return Status(StatusCode::kFailedPrecondition, "no program is loaded");
  }
  program_running_ = true;
  state_.program_running = true;
  BeginMotionLocked();
  return Status::Ok();
}

Status MockDriver::StopProgram() {
  std::lock_guard<std::mutex> lock(mutex_);
  program_running_ = false;
  state_.program_running = false;
  state_.moving = false;
  state_.paused = false;
  jog_active_ = false;
  return Status::Ok();
}

StatusOr<RobotSnapshot> MockDriver::ReadState() {
  std::lock_guard<std::mutex> lock(mutex_);
  RefreshMotionLocked();
  state_.observed_at = std::chrono::steady_clock::now();
  state_.lifecycle = DeriveLifecycleState(state_);
  return state_;
}

StatusOr<std::vector<Alarm>> MockDriver::ReadAlarms() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Alarm> alarms;
  if (state_.alarm_active) {
    alarms.push_back({1, 3, "mock-time", "mock controller alarm"});
  }
  return alarms;
}

StatusOr<bool> MockDriver::ReadDigitalInput(std::uint32_t index) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (index >= digital_inputs_.size()) {
    return Status(StatusCode::kInvalidArgument,
                  "digital input index is out of range");
  }
  return static_cast<bool>(digital_inputs_[index]);
}

StatusOr<bool> MockDriver::ReadDigitalOutput(std::uint32_t index) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (index >= digital_outputs_.size()) {
    return Status(StatusCode::kInvalidArgument,
                  "digital output index is out of range");
  }
  return static_cast<bool>(digital_outputs_[index]);
}

Status MockDriver::WriteDigitalOutput(std::uint32_t index, bool value) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (index >= digital_outputs_.size()) {
    return Status(StatusCode::kInvalidArgument,
                  "digital output index is out of range");
  }
  digital_outputs_[index] = value;
  return Status::Ok();
}

void MockDriver::SetEmergencyStop(bool active) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_.emergency_stop = active;
  if (active) {
    state_.moving = false;
    state_.paused = false;
    state_.servo_on = false;
  }
}

void MockDriver::SetAlarm(bool active) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_.alarm_active = active;
  if (active) {
    state_.moving = false;
  }
}

void MockDriver::SetControllerMode(ControllerMode mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_.controller_mode = mode;
}

Status MockDriver::CheckReadyLocked() const {
  if (!state_.connected || !state_.api_control || !state_.servo_on ||
      state_.emergency_stop || state_.alarm_active || state_.moving ||
      state_.paused) {
    return Status(StatusCode::kFailedPrecondition,
                  "mock robot is not ready for motion");
  }
  return Status::Ok();
}

void MockDriver::BeginMotionLocked() {
  state_.moving = true;
  state_.paused = false;
  jog_active_ = false;
  motion_ends_at_ = std::chrono::steady_clock::now() + motion_duration_;
}

void MockDriver::RefreshMotionLocked() {
  if (state_.moving && !jog_active_ &&
      std::chrono::steady_clock::now() >= motion_ends_at_) {
    state_.moving = false;
    if (program_running_) {
      program_running_ = false;
      state_.program_running = false;
    }
  }
}

}  // namespace robot
