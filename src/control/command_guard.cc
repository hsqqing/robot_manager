// 集中执行命令安全策略，拒绝不满足前置条件的危险操作。
#include "robot/control/command_guard.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

namespace robot {
namespace {

bool IsFinite(double value) { return std::isfinite(value); }

bool IsFinitePose(const CartesianPose& pose) {
  return IsFinite(pose.x_mm) && IsFinite(pose.y_mm) && IsFinite(pose.z_mm) &&
         IsFinite(pose.a_deg) && IsFinite(pose.b_deg) && IsFinite(pose.c_deg) &&
         pose.configuration >= 0;
}

bool IsInsideWorkspace(const CartesianPose& pose,
                       const CartesianWorkspace& workspace) {
  return pose.x_mm >= workspace.minimum_x_mm &&
         pose.x_mm <= workspace.maximum_x_mm &&
         pose.y_mm >= workspace.minimum_y_mm &&
         pose.y_mm <= workspace.maximum_y_mm &&
         pose.z_mm >= workspace.minimum_z_mm &&
         pose.z_mm <= workspace.maximum_z_mm;
}

Status RequireLease(bool lease_valid) {
  if (!lease_valid) {
    return Status(StatusCode::kPermissionDenied,
                  "a valid control lease is required");
  }
  return Status::Ok();
}

Status ValidatePayloadType(const CommandRequest& request) {
  const bool empty = std::holds_alternative<std::monostate>(request.payload);
  bool valid = false;
  switch (request.type) {
    case CommandType::kAcquireControl:
    case CommandType::kReleaseControl:
    case CommandType::kPowerOn:
    case CommandType::kPowerOff:
    case CommandType::kClearFault:
    case CommandType::kHold:
    case CommandType::kResume:
    case CommandType::kStartProgram:
    case CommandType::kStopProgram:
      valid = empty;
      break;
    case CommandType::kSetGlobalSpeed:
      valid = std::holds_alternative<SpeedPayload>(request.payload);
      break;
    case CommandType::kMoveJ:
      valid = std::holds_alternative<MoveJPayload>(request.payload);
      break;
    case CommandType::kMoveL:
      valid = std::holds_alternative<MoveLPayload>(request.payload);
      break;
    case CommandType::kMoveC:
      valid = std::holds_alternative<MoveCPayload>(request.payload);
      break;
    case CommandType::kStop:
      valid = empty || std::holds_alternative<StopPayload>(request.payload);
      break;
    case CommandType::kJog:
      valid = std::holds_alternative<JogCommand>(request.payload);
      break;
    case CommandType::kWriteDigitalOutput:
      valid = std::holds_alternative<DigitalOutputPayload>(request.payload);
      break;
    case CommandType::kLoadProgram:
      valid = std::holds_alternative<ProgramPayload>(request.payload);
      break;
  }
  return valid ? Status::Ok()
               : Status(StatusCode::kInvalidArgument,
                        "command payload has the wrong type");
}

}  // namespace

CommandGuard::CommandGuard(SafetyPolicy policy) : policy_(std::move(policy)) {}

Status CommandGuard::Validate(const CommandRequest& request,
                              const RobotSnapshot& snapshot, bool lease_valid,
                              bool jog_active) const {
  // 此处是所有控制命令的统一软件安全门禁。Stop 保持最小前置条件，
  // 其余命令按类型检查租约、控制器状态、参数和站点配置。
  const Status payload_status = ValidatePayloadType(request);
  if (!payload_status.ok()) {
    return payload_status;
  }

  switch (request.type) {
    case CommandType::kStop:
      return snapshot.connected ? Status::Ok()
                                : Status(StatusCode::kFailedPrecondition,
                                         "robot is not connected");
    case CommandType::kAcquireControl:
      if (!snapshot.connected) {
        return Status(StatusCode::kFailedPrecondition,
                      "robot is not connected");
      }
      return RequireLease(lease_valid);
    case CommandType::kReleaseControl:
      return RequireLease(lease_valid);
    case CommandType::kClearFault:
      if (!snapshot.connected || snapshot.emergency_stop ||
          snapshot.lifecycle == RobotLifecycleState::kUnknown) {
        return Status(StatusCode::kFailedPrecondition,
                      "cannot clear faults while disconnected or in "
                      "emergency stop");
      }
      return RequireLease(lease_valid);
    case CommandType::kPowerOn:
      if (!snapshot.connected || !snapshot.api_control ||
          snapshot.emergency_stop || snapshot.alarm_active ||
          snapshot.lifecycle == RobotLifecycleState::kUnknown) {
        return Status(StatusCode::kFailedPrecondition,
                      "power-on preconditions are not satisfied");
      }
      return RequireLease(lease_valid);
    case CommandType::kPowerOff:
    case CommandType::kStopProgram:
      if (!snapshot.connected || !snapshot.api_control) {
        return Status(StatusCode::kFailedPrecondition,
                      "robot is not under API control");
      }
      break;
    case CommandType::kSetGlobalSpeed:
    case CommandType::kWriteDigitalOutput:
    case CommandType::kLoadProgram:
      if (!snapshot.connected || !snapshot.api_control ||
          snapshot.lifecycle == RobotLifecycleState::kUnknown) {
        return Status(StatusCode::kFailedPrecondition,
                      "robot is not under API control");
      }
      break;
    case CommandType::kStartProgram: {
      const Status common = ValidateCommonMotion(snapshot, lease_valid);
      if (!common.ok()) {
        return common;
      }
      if (!policy_.program_execution_enabled) {
        return Status(StatusCode::kFailedPrecondition,
                      "controller program execution is disabled by policy");
      }
      return Status::Ok();
    }
    case CommandType::kMoveJ: {
      const Status common = ValidateCommonMotion(snapshot, lease_valid);
      if (!common.ok()) {
        return common;
      }
      const auto* payload = std::get_if<MoveJPayload>(&request.payload);
      return payload == nullptr ? Status(StatusCode::kInvalidArgument,
                                         "MoveJ payload is missing")
                                : ValidateMoveJ(*payload);
    }
    case CommandType::kMoveL: {
      const Status common = ValidateCommonMotion(snapshot, lease_valid);
      if (!common.ok()) {
        return common;
      }
      const auto* payload = std::get_if<MoveLPayload>(&request.payload);
      return payload == nullptr ? Status(StatusCode::kInvalidArgument,
                                         "MoveL payload is missing")
                                : ValidateMoveL(*payload);
    }
    case CommandType::kMoveC: {
      const Status common = ValidateCommonMotion(snapshot, lease_valid);
      if (!common.ok()) {
        return common;
      }
      const auto* payload = std::get_if<MoveCPayload>(&request.payload);
      return payload == nullptr ? Status(StatusCode::kInvalidArgument,
                                         "MoveC payload is missing")
                                : ValidateMoveC(*payload);
    }
    case CommandType::kHold:
      if (!snapshot.moving && !snapshot.program_running) {
        return Status(StatusCode::kFailedPrecondition,
                      "robot motion or program is not running");
      }
      return RequireLease(lease_valid);
    case CommandType::kResume:
      if (!snapshot.paused) {
        return Status(StatusCode::kFailedPrecondition,
                      "robot motion is not paused");
      }
      return RequireLease(lease_valid);
    case CommandType::kJog: {
      const auto* jog = std::get_if<JogCommand>(&request.payload);
      const std::size_t configured_axes = policy_.joint_limits.size();
      if (jog == nullptr || jog->axis < 1 ||
          (configured_axes != 0 && jog->axis > configured_axes)) {
        return Status(StatusCode::kInvalidArgument,
                      "jog payload or axis is invalid");
      }
      const Status lease_status = RequireLease(lease_valid);
      if (!lease_status.ok()) {
        return lease_status;
      }
      if (!snapshot.connected || !snapshot.api_control ||
          snapshot.emergency_stop || snapshot.alarm_active ||
          snapshot.lifecycle == RobotLifecycleState::kUnknown ||
          snapshot.program_running ||
          snapshot.controller_mode != ControllerMode::kManual) {
        return Status(StatusCode::kFailedPrecondition,
                      "jog preconditions are not satisfied");
      }
      if (!jog->start) {
        return Status::Ok();
      }
      if (snapshot.moving && !jog_active) {
        return Status(StatusCode::kFailedPrecondition,
                      "cannot start jog during another motion");
      }
      if (!policy_.motion_enabled || !policy_.joint_limits_verified ||
          !snapshot.servo_on || snapshot.paused) {
        return Status(StatusCode::kFailedPrecondition,
                      "jog motion is disabled or robot is not ready");
      }
      if (snapshot.speed_ratio == 0 ||
          policy_.maximum_joint_speed_percent < 1 ||
          snapshot.speed_ratio >
              static_cast<unsigned int>(policy_.maximum_joint_speed_percent)) {
        return Status(StatusCode::kFailedPrecondition,
                      "global speed ratio is unsafe for jog");
      }
      return Status::Ok();
    }
  }

  const Status lease_status = RequireLease(lease_valid);
  if (!lease_status.ok()) {
    return lease_status;
  }

  if (request.type == CommandType::kSetGlobalSpeed) {
    const auto* payload = std::get_if<SpeedPayload>(&request.payload);
    if (payload == nullptr || payload->ratio == 0 ||
        policy_.maximum_joint_speed_percent < 1 ||
        payload->ratio >
            static_cast<unsigned int>(policy_.maximum_joint_speed_percent)) {
      return Status(StatusCode::kInvalidArgument,
                    "global speed ratio exceeds the site policy");
    }
  }
  if (request.type == CommandType::kWriteDigitalOutput) {
    const auto* payload = std::get_if<DigitalOutputPayload>(&request.payload);
    if (payload == nullptr) {
      return Status(StatusCode::kInvalidArgument,
                    "digital output payload is missing");
    }
    const bool allowed =
        std::find(policy_.writable_digital_outputs.begin(),
                  policy_.writable_digital_outputs.end(),
                  payload->index) != policy_.writable_digital_outputs.end();
    if (!allowed) {
      return Status(StatusCode::kPermissionDenied,
                    "digital output is not in the write allowlist");
    }
  }
  if (request.type == CommandType::kLoadProgram) {
    const auto* payload = std::get_if<ProgramPayload>(&request.payload);
    if (payload == nullptr || payload->name.empty()) {
      return Status(StatusCode::kInvalidArgument,
                    "controller program name is required");
    }
    const bool approved =
        std::find(policy_.approved_programs.begin(),
                  policy_.approved_programs.end(),
                  payload->name) != policy_.approved_programs.end();
    if (!policy_.program_execution_enabled || !approved) {
      return Status(StatusCode::kPermissionDenied,
                    "controller program is not approved for execution");
    }
  }
  return Status::Ok();
}

Status CommandGuard::ValidateCommonMotion(const RobotSnapshot& snapshot,
                                          bool lease_valid) const {
  // MoveJ/MoveL/MoveC/启动程序共享的最严格前置条件：自动模式、伺服上电、
  // API 控制权、无故障且没有既有运动。
  const Status lease_status = RequireLease(lease_valid);
  if (!lease_status.ok()) {
    return lease_status;
  }
  if (!policy_.motion_enabled || !policy_.joint_limits_verified) {
    return Status(StatusCode::kFailedPrecondition,
                  "motion or verified site joint limits are disabled");
  }
  if (snapshot.lifecycle != RobotLifecycleState::kReady ||
      snapshot.controller_mode != ControllerMode::kAutomatic ||
      !snapshot.connected || !snapshot.api_control || !snapshot.servo_on ||
      snapshot.emergency_stop || snapshot.alarm_active || snapshot.moving ||
      snapshot.paused || snapshot.program_running) {
    return Status(StatusCode::kFailedPrecondition,
                  "robot is not ready for a new motion command");
  }
  return Status::Ok();
}

Status CommandGuard::ValidateMoveJ(const MoveJPayload& payload) const {
  if (!policy_.joint_limits_verified) {
    return Status(StatusCode::kFailedPrecondition,
                  "site joint limits have not been verified");
  }
  if (payload.target.tool_name.empty() ||
      payload.target.work_object_name.empty() ||
      payload.profile.speed_percent < 1 ||
      payload.profile.speed_percent > policy_.maximum_joint_speed_percent ||
      !IsFinite(payload.profile.blend) || payload.profile.blend < 0.0) {
    return Status(StatusCode::kInvalidArgument,
                  "joint speed or blend is outside the site policy");
  }
  if (payload.target.position.degrees.size() != policy_.joint_limits.size()) {
    return Status(StatusCode::kInvalidArgument,
                  "joint target axis count does not match site limits");
  }
  for (std::size_t index = 0; index < policy_.joint_limits.size(); ++index) {
    const double value = payload.target.position.degrees[index];
    const JointLimit& limit = policy_.joint_limits[index];
    if (!IsFinite(value) || value < limit.minimum_deg ||
        value > limit.maximum_deg) {
      return Status(StatusCode::kInvalidArgument,
                    "joint target is outside the configured limits");
    }
  }
  return Status::Ok();
}

Status CommandGuard::ValidateMoveL(const MoveLPayload& payload) const {
  if (!policy_.cartesian_workspace_verified) {
    return Status(StatusCode::kFailedPrecondition,
                  "site Cartesian workspace has not been verified");
  }
  if (payload.target.tool_name.empty() ||
      payload.target.work_object_name.empty() ||
      payload.target.work_object_name != policy_.cartesian_workspace_frame ||
      !IsFinitePose(payload.target.pose) ||
      !IsInsideWorkspace(payload.target.pose, policy_.cartesian_workspace) ||
      payload.profile.speed_mm_per_second < 1 ||
      payload.profile.speed_mm_per_second >
          policy_.maximum_linear_speed_mm_per_second ||
      !IsFinite(payload.profile.blend_mm) || payload.profile.blend_mm < 0.0) {
    return Status(StatusCode::kInvalidArgument,
                  "linear target or profile is invalid");
  }
  return Status::Ok();
}

Status CommandGuard::ValidateMoveC(const MoveCPayload& payload) const {
  if (!policy_.cartesian_workspace_verified) {
    return Status(StatusCode::kFailedPrecondition,
                  "site Cartesian workspace has not been verified");
  }
  if (payload.target.tool_name.empty() ||
      payload.target.work_object_name.empty() ||
      payload.target.work_object_name != policy_.cartesian_workspace_frame ||
      !IsFinitePose(payload.target.via) ||
      !IsFinitePose(payload.target.target) ||
      !IsInsideWorkspace(payload.target.via, policy_.cartesian_workspace) ||
      !IsInsideWorkspace(payload.target.target, policy_.cartesian_workspace) ||
      payload.profile.speed_mm_per_second < 1 ||
      payload.profile.speed_mm_per_second >
          policy_.maximum_linear_speed_mm_per_second ||
      !IsFinite(payload.profile.blend_mm) || payload.profile.blend_mm < 0.0) {
    return Status(StatusCode::kInvalidArgument,
                  "circular target or profile is invalid");
  }
  return Status::Ok();
}

}  // namespace robot
