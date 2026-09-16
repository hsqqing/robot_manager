// EFORT SDK 适配器：隔离厂商 API、错误码和数据格式。
#include "efort_driver.h"

#include <arpa/inet.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cctype>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "EfortSdk.h"
#include "efort_error_mapper.h"
#include "robot/control/state_machine.h"

namespace robot {
namespace {

constexpr std::size_t kEfortAxisCount = 6;
constexpr std::uint32_t kEfortDigitalIoCount = 176;

std::string NormalizeModelName(const std::string& value) {
  std::string result;
  for (const char c : value) {
    if (std::isalnum(static_cast<unsigned char>(c))) {
      result.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
    }
  }
  return result;
}

bool EfortModelMatches(const std::string& expected, const std::string& actual) {
  const std::string e = NormalizeModelName(expected);
  const std::string a = NormalizeModelName(actual);
  return e.empty() || e == a || (a.rfind("EFORT", 0) == 0 && a.substr(5) == e);
}

std::array<double, kEfortAxisCount> ToEfortPose(const CartesianPose& pose) {
  return {pose.x_mm, pose.y_mm, pose.z_mm, pose.a_deg, pose.b_deg, pose.c_deg};
}

RobotAPI::RobotPos ToRobotPos(const CartesianPose& pose) {
  RobotAPI::RobotPos result;
  result.x = pose.x_mm;
  result.y = pose.y_mm;
  result.z = pose.z_mm;
  result.a = pose.a_deg;
  result.b = pose.b_deg;
  result.c = pose.c_deg;
  result.cfgx = pose.configuration;
  result.cfg1 = pose.joint_1_turn;
  result.cfg4 = pose.joint_4_turn;
  result.cfg6 = pose.joint_6_turn;
  return result;
}

RobotAPI::PointC ToPointC(const  CartesianPose& pose) {
  RobotAPI::PointC result;
  result.x = pose.x_mm;
  result.y = pose.y_mm;
  result.z = pose.z_mm;
  result.a = pose.a_deg;
  result.b = pose.b_deg;
  result.c = pose.c_deg;
  result.cfgx = static_cast<unsigned int>(std::max(pose.configuration, 0));
  result.cfg1 = pose.joint_1_turn;
  result.cfg4 = pose.joint_4_turn;
  result.cfg6 = pose.joint_6_turn;
  return result;
}

std::string SafeString(const char* value, std::size_t maximum_length) {
  const char* end = std::find(value, value + maximum_length, '\0');
  return std::string(value, end);
}

ControllerMode ToControllerMode(RoboxKeyMode mode) {
  switch (mode) {
    case ROBOX_MODE_MANUAL:
      return ControllerMode::kManual;
    case ROBOX_MODE_MANUFAST:
      return ControllerMode::kManualFast;
    case ROBOX_MODE_AUTO:
      return ControllerMode::kAutomatic;
  }
  return ControllerMode::kUnknown;
}

RoboxJogMode ToEfortJogMode(JogFrame frame) {
  switch (frame) {
    case JogFrame::kJoint:
      return ROBOX_MODE_JOG;
    case JogFrame::kRobot:
      return ROBOX_MODE_ROBOT;
    case JogFrame::kTool:
      return ROBOX_MODE_TOOL;
    case JogFrame::kUser:
      return ROBOX_MODE_UFRAME;
    case JogFrame::kWorld:
      return ROBOX_MODE_WORLD;
  }
  return ROBOX_MODE_JOG;
}

}  // namespace

class EfortDriver::Impl {
 public:
  Status Connect(const ConnectionOptions& options) {
    if (connected_) {
      return Status(StatusCode::kAlreadyExists,
                    "EFORT driver is already connected");
    }
    if (options.address.empty()) {
      return Status(StatusCode::kInvalidArgument, "robot address is required");
    }
    if (options.expected_axis_count != 0 &&
        options.expected_axis_count != kEfortAxisCount) {
      return Status(StatusCode::kInvalidArgument,
                    "EFORT adapter requires an axis count of six");
    }
    in_addr parsed_address{};
    if (inet_pton(AF_INET, options.address.c_str(), &parsed_address) != 1) {
      return Status(StatusCode::kInvalidArgument,
                    "EFORT robot address must be an IPv4 address");
    }

    unsigned int device_id = 0;
    const int result = RobotAPI::ConnectRobot(options.address, device_id, false,
                                              options.sdk_debug_logging, 0,
                                              options.verify_sdk_version);
    const Status status = MapEfortError(result, "ConnectRobot");
    if (!status.ok()) {
      return status;
    }

    device_id_ = device_id;
    expected_model_ = options.expected_model;
    connected_ = true;

    std::string controller_model;
    const Status model_status = MapEfortError(
        RobotAPI::GetCurrentRobotType(controller_model, device_id_),
        "GetCurrentRobotType");
    if (!model_status.ok()) {
      Disconnect();
      return model_status;
    }
    if (!EfortModelMatches(expected_model_, controller_model)) {
      const std::string expected = expected_model_;
      Disconnect();
      return Status(StatusCode::kFailedPrecondition,
                    "configured model " + expected +
                        " does not match controller model " + controller_model);
    }
    controller_model_ = controller_model;
    return Status::Ok();
  }

  Status Disconnect() {
    if (!connected_) {
      return Status::Ok();
    }
    StopActiveJog();
    if (RobotAPI::IsApiControl(device_id_)) {
      RobotAPI::EnableApiControl(false, device_id_);
    }
    RobotAPI::DisconnectRobot(device_id_);
    device_id_ = 0;
    connected_ = false;
    paused_by_service_ = false;
    has_active_jog_ = false;
    controller_model_.clear();
    return Status::Ok();
  }

  Status AcquireControl() {
    return MapEfortError(RobotAPI::EnableApiControl(true, device_id_),
                         "EnableApiControl(true)");
  }

  Status ReleaseControl() {
    return MapEfortError(RobotAPI::EnableApiControl(false, device_id_),
                         "EnableApiControl(false)");
  }

  Status PowerOn() {
    return MapEfortError(RobotAPI::PowerOn(device_id_), "PowerOn");
  }

  Status PowerOff() {
    const Status jog_stop = StopActiveJog();
    if (!jog_stop.ok()) {
      return jog_stop;
    }
    paused_by_service_ = false;
    return MapEfortError(RobotAPI::PowerOff(device_id_), "PowerOff");
  }

  Status ClearFault() {
    bool emergency_stop = false;
    const Status emergency_status =
        MapEfortError(RobotAPI::GetCurrentEmgStatus(emergency_stop, device_id_),
                      "GetCurrentEmgStatus");
    if (!emergency_status.ok()) {
      return emergency_status;
    }
    if (emergency_stop) {
      return Status(StatusCode::kFailedPrecondition,
                    "cannot clear alarms while emergency stop is active");
    }
    return MapEfortError(RobotAPI::ClearAlarm(device_id_), "ClearAlarm");
  }

  Status SetGlobalSpeed(unsigned int ratio) {
    if (ratio < 1 || ratio > 100) {
      return Status(StatusCode::kInvalidArgument,
                    "speed ratio must be in [1, 100]");
    }
    return MapEfortError(RobotAPI::SetGlobalSpeed(ratio, device_id_),
                         "SetGlobalSpeed");
  }

  Status MoveJ(const JointTarget& target, const JointMotionProfile& profile) {
    RobotAPI::RobotJoint check_target;
    if (target.position.degrees.size() != kEfortAxisCount) {
      return Status(StatusCode::kInvalidArgument, "EFORT MoveJ requires six joint values");
    }
    for (std::size_t index = 0; index < kEfortAxisCount; ++index) {
      check_target.j[index] = target.position.degrees[index];
    }
    Status status = MapEfortError(
        RobotAPI::CheckTarget(check_target, target.tool_name,
                              target.work_object_name, device_id_),
        "CheckTarget(MoveJ)");
    if (!status.ok()) {
      return status;
    }

    std::array<double, kEfortAxisCount> position{};
    std::copy(target.position.degrees.begin(), target.position.degrees.end(), position.begin());
    paused_by_service_ = false;
    return MapEfortError(
        RobotAPI::MJOINT(position.data(), profile.speed_percent, profile.blend,
                         device_id_, 10, PRINT_NONE),
        "MJOINT");
  }

  Status MoveL(const CartesianTarget& target,
               const CartesianMotionProfile& profile) {
    Status status = SelectFrames(target.tool_name, target.work_object_name);
    if (!status.ok()) {
      return status;
    }
    const RobotAPI::RobotPos check_target = ToRobotPos(target.pose);
    status = MapEfortError(
        RobotAPI::CheckTarget(check_target, target.tool_name,
                              target.work_object_name, device_id_),
        "CheckTarget(MoveL)");
    if (!status.ok()) {
      return status;
    }

  std::array<double, kEfortAxisCount> position = ToEfortPose(target.pose);
    paused_by_service_ = false;
    return MapEfortError(
        RobotAPI::MLIN(position.data(), profile.speed_mm_per_second,
                       profile.blend_mm, device_id_, 10, PRINT_NONE),
        "MLIN");
  }

  Status MoveC(const CircularTarget& target,
               const CartesianMotionProfile& profile) {
    Status status = SelectFrames(target.tool_name, target.work_object_name);
    if (!status.ok()) {
      return status;
    }
    const RobotAPI::RobotPos check_via = ToRobotPos(target.via);
    const RobotAPI::RobotPos check_target = ToRobotPos(target.target);
    status = MapEfortError(
        RobotAPI::CheckTarget(check_via, target.tool_name,
                              target.work_object_name, device_id_),
        "CheckTarget(MoveC via)");
    if (!status.ok()) {
      return status;
    }
    status = MapEfortError(
        RobotAPI::CheckTarget(check_target, target.tool_name,
                              target.work_object_name, device_id_),
        "CheckTarget(MoveC target)");
    if (!status.ok()) {
      return status;
    }

    paused_by_service_ = false;
    return MapEfortError(
        RobotAPI::MCIRC(ToPointC(target.via), ToPointC(target.target),
                        static_cast<double>(profile.speed_mm_per_second),
                        profile.blend_mm, target.tool_name,
                        target.work_object_name, device_id_),
        "MCIRC");
  }

  Status Hold() {
    const Status status =
        MapEfortError(RobotAPI::MOVEHOLD(device_id_), "MOVEHOLD");
    if (status.ok()) {
      paused_by_service_ = true;
    }
    return status;
  }

  Status Resume() {
    const Status status =
        MapEfortError(RobotAPI::MOVERESUME(device_id_), "MOVERESUME");
    if (status.ok()) {
      paused_by_service_ = false;
    }
    return status;
  }

  Status Stop(StopMode mode) {
    if (mode == StopMode::kQuick) {
      return Status(StatusCode::kFailedPrecondition,
                    "EFORT SDK V2.8 has no distinct quick-stop API");
    }
    const Status jog_stop = StopActiveJog();
    if (!jog_stop.ok()) {
      return jog_stop;
    }
    paused_by_service_ = false;
    if (mode == StopMode::kPowerOffRequest) {
      return PowerOff();
    }

    RoboxProgramState program_state = RPL_INIT;
    const int program_result =
        RobotAPI::GetCurrentProStatus(program_state, device_id_);
    const Status program_state_status =
        MapEfortError(program_result, "GetCurrentProStatus(stop)");
    if (!program_state_status.ok()) {
      return program_state_status;
    }
    if (program_state == RPL_RUN || program_state == RPL_PAUSE) {
      const Status program_stop =
          MapEfortError(RobotAPI::StopProgram(device_id_), "StopProgram");
      if (!program_stop.ok()) {
        return program_stop;
      }
    }

    bool moving = false;
    const int moving_result = RobotAPI::GetMoveState(moving, device_id_);
    const Status moving_status =
        MapEfortError(moving_result, "GetMoveState(stop)");
    if (!moving_status.ok()) {
      return moving_status;
    }
    if (moving) {
      const Status hold =
          MapEfortError(RobotAPI::MOVEHOLD(device_id_), "MOVEHOLD");
      if (!hold.ok()) {
        return hold;
      }
    }
    return MapEfortError(RobotAPI::MOVECLEAR(device_id_), "MOVECLEAR");
  }

  Status Jog(const JogCommand& command) {
    if (command.axis < 1 || command.axis > kEfortAxisCount) {
      return Status(StatusCode::kInvalidArgument, "jog axis must be in [1, 6]");
    }
    if (!command.start) {
      return StopActiveJog();
    }
    if (has_active_jog_ && active_jog_.frame == command.frame &&
        active_jog_.axis == command.axis &&
        active_jog_.direction == command.direction) {
      return Status::Ok();
    }

    Status status = StopActiveJog();
    if (!status.ok()) {
      return status;
    }
    status = MapEfortError(
        RobotAPI::SetJogMode(static_cast<int>(ToEfortJogMode(command.frame)),
                             device_id_),
        "SetJogMode");
    if (!status.ok()) {
      return status;
    }
    status = CallJog(command, true);
    if (status.ok()) {
      active_jog_ = command;
      has_active_jog_ = true;
    }
    return status;
  }

  Status LoadProgram(const std::string& name) {
    if (name.empty()) {
      return Status(StatusCode::kInvalidArgument, "program name is required");
    }
    return MapEfortError(RobotAPI::LoadProgram(name, device_id_, false),
                         "LoadProgram");
  }

  Status StartProgram() {
    paused_by_service_ = false;
    return MapEfortError(RobotAPI::StartProgram(device_id_), "StartProgram");
  }

  Status StopProgram() {
    const Status status =
        MapEfortError(RobotAPI::StopProgram(device_id_), "StopProgram");
    if (status.ok()) {
      paused_by_service_ = false;
    }
    return status;
  }

  StatusOr<RobotSnapshot> ReadState() {
    RobotSnapshot snapshot;
    snapshot.observed_at = std::chrono::steady_clock::now();
    snapshot.connected = connected_ && RobotAPI::IsConnected(device_id_);
    if (!snapshot.connected) {
      snapshot.lifecycle = RobotLifecycleState::kDisconnected;
      return snapshot;
    }

    RobotAPI::RunInfo run_info{};
    Status status =
        MapEfortError(RobotAPI::GetRobotStatusData(run_info, device_id_),
                      "GetRobotStatusData");
    if (!status.ok()) {
      return status;
    }
    snapshot.api_control = RobotAPI::IsApiControl(device_id_);
    snapshot.controller_mode = ToControllerMode(run_info.keyMode);
    snapshot.speed_ratio = run_info.velocity;
    snapshot.tcp_speed_mm_per_second = run_info.tcpSpeed;
    snapshot.program_running = run_info.programStatus == RPL_RUN;
    const bool program_error = run_info.programStatus == RPL_ERROR;
    snapshot.active_tool = SafeString(run_info.toolName, MAX_CHAR_LENGTH);
    snapshot.active_work_object =
        SafeString(run_info.wobjName, MAX_CHAR_LENGTH);
    snapshot.controller_model = controller_model_;

    status = MapEfortError(
        RobotAPI::GetCurrentServoStatus(snapshot.servo_on, device_id_),
        "GetCurrentServoStatus");
    if (!status.ok()) {
      return status;
    }
    status = MapEfortError(
        RobotAPI::GetCurrentEmgStatus(snapshot.emergency_stop, device_id_),
        "GetCurrentEmgStatus");
    if (!status.ok()) {
      return status;
    }
    status = MapEfortError(
        RobotAPI::GetCurrentAlarmStatus(snapshot.alarm_active, device_id_),
        "GetCurrentAlarmStatus");
    if (!status.ok()) {
      return status;
    }
    snapshot.alarm_active = snapshot.alarm_active || program_error;
    status = MapEfortError(RobotAPI::GetMoveState(snapshot.moving, device_id_),
                           "GetMoveState");
    if (!status.ok()) {
      return status;
    }
    snapshot.paused = paused_by_service_ || run_info.programStatus == RPL_PAUSE;

    RobotAPI::RobotJoint joints;
    status =
        MapEfortError(RobotAPI::GetJointPos(joints, device_id_), "GetJointPos");
    if (!status.ok()) {
      return status;
    }
    snapshot.axis_count = kEfortAxisCount;
    snapshot.joints.degrees.resize(kEfortAxisCount);
    for (std::size_t index = 0; index < kEfortAxisCount; ++index) {
      snapshot.joints.degrees[index] = joints.j[index];
    }

    RobotAPI::RobotPos pose;
    status = MapEfortError(RobotAPI::GetBaseCoordinatePos2(pose, device_id_),
                           "GetBaseCoordinatePos2");
    if (!status.ok()) {
      return status;
    }
    snapshot.tcp_pose = {pose.x, pose.y,    pose.z,    pose.a,    pose.b,
                         pose.c, pose.cfgx, pose.cfg1, pose.cfg4, pose.cfg6};
    snapshot.lifecycle = DeriveLifecycleState(snapshot);
    return snapshot;
  }

  StatusOr<std::vector<Alarm>> ReadAlarms() {
    std::vector<RobotAPI::AlarmData> vendor_alarms;
    const Status status = MapEfortError(
        RobotAPI::GetAlarmData(ALA_CURRENT, vendor_alarms, device_id_),
        "GetAlarmData");
    if (!status.ok()) {
      return status;
    }
    std::vector<Alarm> alarms;
    alarms.reserve(vendor_alarms.size());
    for (const RobotAPI::AlarmData& vendor_alarm : vendor_alarms) {
      alarms.push_back({vendor_alarm.alarmCode,
                        static_cast<int>(vendor_alarm.alarmType),
                        SafeString(vendor_alarm.alarmTime, MAX_CHAR_LENGTH),
                        SafeString(vendor_alarm.alarmInfo, MAX_CHAR_LENGTH)});
    }
    return alarms;
  }

  StatusOr<bool> ReadDigitalInput(std::uint32_t index) {
    if (index >= kEfortDigitalIoCount) {
      return Status(StatusCode::kInvalidArgument,
                    "digital input index must be in [0, 175]");
    }
    bool value = false;
    const Status status =
        MapEfortError(RobotAPI::ReadDIn(index, value, device_id_), "ReadDIn");
    return status.ok() ? StatusOr<bool>(value) : StatusOr<bool>(status);
  }

  StatusOr<bool> ReadDigitalOutput(std::uint32_t index) {
    if (index >= kEfortDigitalIoCount) {
      return Status(StatusCode::kInvalidArgument,
                    "digital output index must be in [0, 175]");
    }
    bool value = false;
    const Status status =
        MapEfortError(RobotAPI::ReadDOut(index, value, device_id_), "ReadDOut");
    return status.ok() ? StatusOr<bool>(value) : StatusOr<bool>(status);
  }

  Status WriteDigitalOutput(std::uint32_t index, bool value) {
    if (index >= kEfortDigitalIoCount) {
      return Status(StatusCode::kInvalidArgument,
                    "digital output index must be in [0, 175]");
    }
    return MapEfortError(RobotAPI::WriteDOut(index, value, device_id_, false),
                         "WriteDOut");
  }

 private:
  Status SelectFrames(const std::string& tool_name,
                      const std::string& work_object_name) {
    Status status =
        MapEfortError(RobotAPI::SetCurrentToolByName(tool_name, device_id_),
                      "SetCurrentToolByName");
    if (!status.ok()) {
      return status;
    }
    return MapEfortError(
        RobotAPI::SetCurrentUframeByName(work_object_name, device_id_),
        "SetCurrentUframeByName");
  }

  Status CallJog(const JogCommand& command, bool start) {
    const bool positive = command.direction == JogDirection::kPositive;
    int result = ERROR_PARA_INVALID;
    switch (command.axis) {
      case 1:
        result = positive ? RobotAPI::Jog1Plus(start, device_id_)
                          : RobotAPI::Jog1Minus(start, device_id_);
        break;
      case 2:
        result = positive ? RobotAPI::Jog2Plus(start, device_id_)
                          : RobotAPI::Jog2Minus(start, device_id_);
        break;
      case 3:
        result = positive ? RobotAPI::Jog3Plus(start, device_id_)
                          : RobotAPI::Jog3Minus(start, device_id_);
        break;
      case 4:
        result = positive ? RobotAPI::Jog4Plus(start, device_id_)
                          : RobotAPI::Jog4Minus(start, device_id_);
        break;
      case 5:
        result = positive ? RobotAPI::Jog5Plus(start, device_id_)
                          : RobotAPI::Jog5Minus(start, device_id_);
        break;
      case 6:
        result = positive ? RobotAPI::Jog6Plus(start, device_id_)
                          : RobotAPI::Jog6Minus(start, device_id_);
        break;
      default:
        break;
    }
    if (!start && has_active_jog_ && active_jog_.axis == command.axis &&
        active_jog_.direction == command.direction) {
      has_active_jog_ = false;
    }
    return MapEfortError(result, start ? "Jog(start)" : "Jog(stop)");
  }

  Status StopActiveJog() {
    if (!has_active_jog_) {
      return Status::Ok();
    }
    const JogCommand active = active_jog_;
    const Status status = CallJog(active, false);
    if (status.ok()) {
      has_active_jog_ = false;
    }
    return status;
  }

  unsigned int device_id_ = 0;
  bool connected_ = false;
  bool paused_by_service_ = false;
  bool has_active_jog_ = false;
  std::string expected_model_;
  JogCommand active_jog_;
  std::string controller_model_;
};

EfortDriver::EfortDriver() : impl_(std::make_unique<Impl>()) {}

EfortDriver::~EfortDriver() { impl_->Disconnect(); }

Status EfortDriver::Connect(const ConnectionOptions& options) {
  return impl_->Connect(options);
}

Status EfortDriver::Disconnect() { return impl_->Disconnect(); }

Status EfortDriver::AcquireControl() { return impl_->AcquireControl(); }

Status EfortDriver::ReleaseControl() { return impl_->ReleaseControl(); }

Status EfortDriver::PowerOn() { return impl_->PowerOn(); }

Status EfortDriver::PowerOff() { return impl_->PowerOff(); }

Status EfortDriver::ClearFault() { return impl_->ClearFault(); }

Status EfortDriver::SetGlobalSpeed(unsigned int ratio) {
  return impl_->SetGlobalSpeed(ratio);
}

Status EfortDriver::MoveJ(const JointTarget& target,
                               const JointMotionProfile& profile) {
  return impl_->MoveJ(target, profile);
}

Status EfortDriver::MoveL(const CartesianTarget& target,
                               const CartesianMotionProfile& profile) {
  return impl_->MoveL(target, profile);
}

Status EfortDriver::MoveC(const CircularTarget& target,
                               const CartesianMotionProfile& profile) {
  return impl_->MoveC(target, profile);
}

Status EfortDriver::Hold() { return impl_->Hold(); }

Status EfortDriver::Resume() { return impl_->Resume(); }

Status EfortDriver::Stop(StopMode mode) { return impl_->Stop(mode); }

Status EfortDriver::Jog(const JogCommand& command) {
  return impl_->Jog(command);
}

Status EfortDriver::LoadProgram(const std::string& name) {
  return impl_->LoadProgram(name);
}

Status EfortDriver::StartProgram() { return impl_->StartProgram(); }

Status EfortDriver::StopProgram() { return impl_->StopProgram(); }

StatusOr<RobotSnapshot> EfortDriver::ReadState() {
  return impl_->ReadState();
}

StatusOr<std::vector<Alarm>> EfortDriver::ReadAlarms() {
  return impl_->ReadAlarms();
}

StatusOr<bool> EfortDriver::ReadDigitalInput(std::uint32_t index) {
  return impl_->ReadDigitalInput(index);
}

StatusOr<bool> EfortDriver::ReadDigitalOutput(std::uint32_t index) {
  return impl_->ReadDigitalOutput(index);
}

Status EfortDriver::WriteDigitalOutput(std::uint32_t index, bool value) {
  return impl_->WriteDigitalOutput(index, value);
}

}  // namespace robot
