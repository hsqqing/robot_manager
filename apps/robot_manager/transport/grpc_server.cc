// gRPC 适配器：将协议请求映射到 RobotService，并处理 TLS 与停机。
#include "robot_manager/transport/grpc_server.h"

#include <grpcpp/grpcpp.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "robot/v1/robot_control.grpc.pb.h"

namespace robot {
namespace {

StatusOr<std::string> ReadFile(const std::string& path) {
  // 证书文件在启动阶段一次性读入内存；读取失败直接阻止服务启动，
  // 避免服务器以“看似启动、实际无法建立安全连接”的状态运行。
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return Status(StatusCode::kNotFound,
                  "unable to read gRPC credential file: " + path);
  }
  return std::string(std::istreambuf_iterator<char>(file),
                     std::istreambuf_iterator<char>());
}

bool IsLoopbackListenAddress(const std::string& address) {
  // 明文 gRPC 只允许绑定本机回环地址，防止开发配置意外暴露到生产网络。
  return address.rfind("127.", 0) == 0 || address.rfind("localhost:", 0) == 0 ||
         address.rfind("[::1]:", 0) == 0 || address.rfind("unix:", 0) == 0;
}

::grpc::Status ToGrpcStatus(const Status& status) {
  // 领域层使用自己的 StatusCode；在传输边界统一映射为 gRPC 标准状态码。
  if (status.ok()) {
    return ::grpc::Status::OK;
  }
  ::grpc::StatusCode code = ::grpc::StatusCode::INTERNAL;
  switch (status.code()) {
    case StatusCode::kOk:
      code = ::grpc::StatusCode::OK;
      break;
    case StatusCode::kInvalidArgument:
      code = ::grpc::StatusCode::INVALID_ARGUMENT;
      break;
    case StatusCode::kFailedPrecondition:
    case StatusCode::kControllerFault:
      code = ::grpc::StatusCode::FAILED_PRECONDITION;
      break;
    case StatusCode::kPermissionDenied:
      code = ::grpc::StatusCode::PERMISSION_DENIED;
      break;
    case StatusCode::kNotFound:
      code = ::grpc::StatusCode::NOT_FOUND;
      break;
    case StatusCode::kAlreadyExists:
      code = ::grpc::StatusCode::ALREADY_EXISTS;
      break;
    case StatusCode::kResourceExhausted:
      code = ::grpc::StatusCode::RESOURCE_EXHAUSTED;
      break;
    case StatusCode::kUnavailable:
      code = ::grpc::StatusCode::UNAVAILABLE;
      break;
    case StatusCode::kDeadlineExceeded:
      code = ::grpc::StatusCode::DEADLINE_EXCEEDED;
      break;
    case StatusCode::kInternal:
      code = ::grpc::StatusCode::INTERNAL;
      break;
  }
  return ::grpc::Status(code, status.message());
}

robot::v1::CommandState ToProtoCommandState(CommandState state) {
  switch (state) {
    case CommandState::kQueued:
      return robot::v1::COMMAND_STATE_QUEUED;
    case CommandState::kRunning:
      return robot::v1::COMMAND_STATE_RUNNING;
    case CommandState::kSucceeded:
      return robot::v1::COMMAND_STATE_SUCCEEDED;
    case CommandState::kFailed:
      return robot::v1::COMMAND_STATE_FAILED;
    case CommandState::kCancelled:
      return robot::v1::COMMAND_STATE_CANCELLED;
    case CommandState::kTimedOut:
      return robot::v1::COMMAND_STATE_TIMED_OUT;
    case CommandState::kStopped:
      return robot::v1::COMMAND_STATE_STOPPED;
  }
  return robot::v1::COMMAND_STATE_UNSPECIFIED;
}

robot::v1::LifecycleState ToProtoLifecycleState(RobotLifecycleState state) {
  switch (state) {
    case RobotLifecycleState::kDisconnected:
      return robot::v1::LIFECYCLE_STATE_DISCONNECTED;
    case RobotLifecycleState::kConnecting:
      return robot::v1::LIFECYCLE_STATE_CONNECTING;
    case RobotLifecycleState::kStandby:
      return robot::v1::LIFECYCLE_STATE_STANDBY;
    case RobotLifecycleState::kReady:
      return robot::v1::LIFECYCLE_STATE_READY;
    case RobotLifecycleState::kExecuting:
      return robot::v1::LIFECYCLE_STATE_EXECUTING;
    case RobotLifecycleState::kPaused:
      return robot::v1::LIFECYCLE_STATE_PAUSED;
    case RobotLifecycleState::kFault:
      return robot::v1::LIFECYCLE_STATE_FAULT;
    case RobotLifecycleState::kEmergencyStop:
      return robot::v1::LIFECYCLE_STATE_EMERGENCY_STOP;
    case RobotLifecycleState::kUnknown:
      return robot::v1::LIFECYCLE_STATE_UNKNOWN;
  }
  return robot::v1::LIFECYCLE_STATE_UNKNOWN;
}

robot::v1::ControllerMode ToProtoControllerMode(ControllerMode mode) {
  switch (mode) {
    case ControllerMode::kManual:
      return robot::v1::CONTROLLER_MODE_MANUAL;
    case ControllerMode::kManualFast:
      return robot::v1::CONTROLLER_MODE_MANUAL_FAST;
    case ControllerMode::kAutomatic:
      return robot::v1::CONTROLLER_MODE_AUTOMATIC;
    case ControllerMode::kUnknown:
      return robot::v1::CONTROLLER_MODE_UNKNOWN;
  }
  return robot::v1::CONTROLLER_MODE_UNKNOWN;
}

CartesianPose FromProtoPose(const robot::v1::CartesianPose& pose) {
  return {pose.x_mm(),          pose.y_mm(),         pose.z_mm(),
          pose.a_deg(),         pose.b_deg(),        pose.c_deg(),
          pose.configuration(), pose.joint_1_turn(), pose.joint_4_turn(),
          pose.joint_6_turn()};
}

void PopulatePose(const CartesianPose& source,
                  robot::v1::CartesianPose* destination) {
  destination->set_x_mm(source.x_mm);
  destination->set_y_mm(source.y_mm);
  destination->set_z_mm(source.z_mm);
  destination->set_a_deg(source.a_deg);
  destination->set_b_deg(source.b_deg);
  destination->set_c_deg(source.c_deg);
  destination->set_configuration(source.configuration);
  destination->set_joint_1_turn(source.joint_1_turn);
  destination->set_joint_4_turn(source.joint_4_turn);
  destination->set_joint_6_turn(source.joint_6_turn);
}

void PopulateState(const std::string& robot_id, const RobotSnapshot& source,
                   robot::v1::RobotState* destination) {
  // 将内部不可变状态快照复制为 protobuf。steady_clock 只适合计算年龄，
  // 因此 observed_at 还要换算为 wall-clock Timestamp 供客户端展示。
  destination->set_robot_id(robot_id);
  destination->set_sequence(source.sequence);
  destination->set_lifecycle(ToProtoLifecycleState(source.lifecycle));
  destination->set_connected(source.connected);
  destination->set_api_control(source.api_control);
  destination->set_servo_on(source.servo_on);
  destination->set_emergency_stop(source.emergency_stop);
  destination->set_alarm_active(source.alarm_active);
  destination->set_moving(source.moving);
  destination->set_paused(source.paused);
  destination->set_program_running(source.program_running);
  destination->set_controller_mode(
      ToProtoControllerMode(source.controller_mode));
  const auto steady_now = std::chrono::steady_clock::now();
  std::uint64_t state_age_ms = 0;
  if (source.observed_at != std::chrono::steady_clock::time_point{} &&
      source.observed_at <= steady_now) {
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
        steady_now - source.observed_at);
    state_age_ms = static_cast<std::uint64_t>(age.count());
  }
  destination->set_state_age_ms(state_age_ms);
  destination->set_state_stale(source.lifecycle ==
                               RobotLifecycleState::kUnknown);
  destination->set_speed_ratio(source.speed_ratio);
  destination->set_tcp_speed_mm_per_second(source.tcp_speed_mm_per_second);
  destination->set_controller_model(source.controller_model);
  destination->set_active_tool(source.active_tool);
  destination->set_active_work_object(source.active_work_object);
  for (double joint : source.joints.degrees) {
    destination->mutable_joints()->add_degrees(joint);
  }
  PopulatePose(source.tcp_pose, destination->mutable_tcp_pose());

  const auto timestamp_now = std::chrono::steady_clock::now();
  auto observed_at = std::chrono::system_clock::now();
  if (source.observed_at != std::chrono::steady_clock::time_point{} &&
      source.observed_at <= timestamp_now) {
    observed_at -=
        std::chrono::duration_cast<std::chrono::system_clock::duration>(
            timestamp_now - source.observed_at);
  }
  const auto now = observed_at.time_since_epoch();
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
  const auto nanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now - seconds);
  destination->mutable_observed_at()->set_seconds(seconds.count());
  destination->mutable_observed_at()->set_nanos(
      static_cast<std::int32_t>(nanoseconds.count()));
}

class RobotControlEndpoint final
    : public robot::v1::RobotControlService::Service {
 public:
  RobotControlEndpoint(std::string robot_id, RobotService* service,
                       std::vector<std::string> allowed_client_common_names)
      : robot_id_(std::move(robot_id)),
        service_(service),
        allowed_client_common_names_(std::move(allowed_client_common_names)) {}

  ::grpc::Status AcquireControlLease(
      ::grpc::ServerContext* context,
      const robot::v1::AcquireControlLeaseRequest* request,
      robot::v1::ControlLease* response) override {
    // 每个 RPC 都先做客户端证书鉴权，再确认 robot_id 属于本实例。
    const ::grpc::Status status = ValidateRequest(*context, request->robot_id());
    if (!status.ok()) {
      return status;
    }
    StatusOr<ControlLease> lease =
        service_->AcquireControlLease(request->client_id());
    if (!lease.ok()) {
      return ToGrpcStatus(lease.status());
    }
    PopulateLease(lease.value(), response);
    return ::grpc::Status::OK;
  }

  ::grpc::Status RenewControlLease(
      ::grpc::ServerContext* context,
      const robot::v1::RenewControlLeaseRequest* request,
      robot::v1::ControlLease* response) override {
    const ::grpc::Status status = ValidateRequest(*context, request->robot_id());
    if (!status.ok()) {
      return status;
    }
    StatusOr<ControlLease> lease =
        service_->RenewControlLease(request->lease_id(), request->client_id());
    if (!lease.ok()) {
      return ToGrpcStatus(lease.status());
    }
    PopulateLease(lease.value(), response);
    return ::grpc::Status::OK;
  }

  ::grpc::Status ReleaseControlLease(
      ::grpc::ServerContext* context,
      const robot::v1::ReleaseControlLeaseRequest* request,
      google::protobuf::Empty*) override {
    const ::grpc::Status status = ValidateRequest(*context, request->robot_id());
    if (!status.ok()) {
      return status;
    }
    return ToGrpcStatus(service_->ReleaseControlLease(request->lease_id(),
                                                       request->client_id()));
  }

  ::grpc::Status SubmitCommand(::grpc::ServerContext* context,
                               const robot::v1::CommandRequest* request,
                               robot::v1::CommandAccepted* response) override {
    const ::grpc::Status status = ValidateRequest(*context, request->robot_id());
    if (!status.ok()) {
      return status;
    }
    // gRPC 线程只负责校验和入队，不直接调用厂商 SDK；真正执行由
    // CommandExecutor 的单写者线程串行完成。
    StatusOr<CommandRequest> command = FromProtoCommand(*request);
    if (!command.ok()) {
      return ToGrpcStatus(command.status());
    }
    StatusOr<std::uint64_t> command_id =
        service_->Submit(std::move(command.value()));
    if (!command_id.ok()) {
      return ToGrpcStatus(command_id.status());
    }
    response->set_command_id(command_id.value());
    return ::grpc::Status::OK;
  }

  ::grpc::Status GetCommand(::grpc::ServerContext* context,
                            const robot::v1::GetCommandRequest* request,
                            robot::v1::Command* response) override {
    const ::grpc::Status status = ValidateRequest(*context, request->robot_id());
    if (!status.ok()) {
      return status;
    }
    const StatusOr<CommandRecord> command =
        service_->GetCommand(request->command_id());
    if (!command.ok()) {
      return ToGrpcStatus(command.status());
    }
    response->set_command_id(command.value().id);
    response->set_state(ToProtoCommandState(command.value().state));
    response->set_error_message(command.value().result.message());
    response->set_vendor_error_code(command.value().result.vendor_code());
    return ::grpc::Status::OK;
  }

  ::grpc::Status GetState(::grpc::ServerContext* context,
                          const robot::v1::GetStateRequest* request,
                          robot::v1::RobotState* response) override {
    const ::grpc::Status status = ValidateRequest(*context, request->robot_id());
    if (!status.ok()) {
      return status;
    }
    PopulateState(robot_id_, service_->GetState(), response);
    return ::grpc::Status::OK;
  }

  ::grpc::Status StreamState(
      ::grpc::ServerContext* context,
      const robot::v1::StreamStateRequest* request,
      ::grpc::ServerWriter<robot::v1::RobotState>* writer) override {
    const ::grpc::Status status = ValidateRequest(*context, request->robot_id());
    if (!status.ok()) {
      return status;
    }
    // 服务端流按客户端要求推送状态，限制在 1~50Hz，避免单个客户端
    // 过度占用 CPU/网络；客户端断开后 IsCancelled() 使循环退出。
    const std::uint32_t rate =
        request->maximum_rate_hz() == 0
            ? 20U
            : std::clamp(request->maximum_rate_hz(), 1U, 50U);
    const auto interval = std::chrono::milliseconds(1000 / rate);
    while (!context->IsCancelled()) {
      robot::v1::RobotState state;
      PopulateState(robot_id_, service_->GetState(), &state);
      if (!writer->Write(state)) {
        break;
      }
      std::this_thread::sleep_for(interval);
    }
    return ::grpc::Status::OK;
  }

  ::grpc::Status ReadAlarms(::grpc::ServerContext* context,
                            const robot::v1::GetStateRequest* request,
                            robot::v1::AlarmList* response) override {
    const ::grpc::Status status = ValidateRequest(*context, request->robot_id());
    if (!status.ok()) {
      return status;
    }
    const StatusOr<std::vector<Alarm>> alarms = service_->ReadAlarms();
    if (!alarms.ok()) {
      return ToGrpcStatus(alarms.status());
    }
    for (const Alarm& alarm : alarms.value()) {
      robot::v1::Alarm* destination = response->add_alarms();
      destination->set_code(alarm.code);
      destination->set_severity(alarm.severity);
      destination->set_occurred_at(alarm.occurred_at);
      destination->set_message(alarm.message);
    }
    return ::grpc::Status::OK;
  }

  ::grpc::Status ReadDigitalInput(
      ::grpc::ServerContext* context,
      const robot::v1::ReadDigitalIoRequest* request,
      robot::v1::DigitalIoValue* response) override {
    const ::grpc::Status status = ValidateRequest(*context, request->robot_id());
    if (!status.ok()) {
      return status;
    }
    const StatusOr<bool> value = service_->ReadDigitalInput(request->index());
    if (!value.ok()) {
      return ToGrpcStatus(value.status());
    }
    response->set_index(request->index());
    response->set_value(value.value());
    return ::grpc::Status::OK;
  }

  ::grpc::Status ReadDigitalOutput(
      ::grpc::ServerContext* context,
      const robot::v1::ReadDigitalIoRequest* request,
      robot::v1::DigitalIoValue* response) override {
    const ::grpc::Status status = ValidateRequest(*context, request->robot_id());
    if (!status.ok()) {
      return status;
    }
    const StatusOr<bool> value = service_->ReadDigitalOutput(request->index());
    if (!value.ok()) {
      return ToGrpcStatus(value.status());
    }
    response->set_index(request->index());
    response->set_value(value.value());
    return ::grpc::Status::OK;
  }

 private:
  ::grpc::Status ValidateRequest(const ::grpc::ServerContext& context,
                                 const std::string& robot_id) const {
    const ::grpc::Status status = Authorize(context);
    return status.ok() ? ValidateRobotId(robot_id) : status;
  }

  ::grpc::Status Authorize(const ::grpc::ServerContext& context) const {
    // allowlist 为空表示由启动配置保证的“无需额外 CN 筛选”模式；
    // 生产 mTLS 通常配置 CN 白名单，只有命中的证书身份才能控制机器人。
    if (allowed_client_common_names_.empty()) {
      return ::grpc::Status::OK;
    }
    const std::shared_ptr<const ::grpc::AuthContext> auth_context =
        context.auth_context();
    if (auth_context == nullptr) {
      return ::grpc::Status(::grpc::StatusCode::UNAUTHENTICATED,
                            "mTLS client certificate is required");
    }
    for (const grpc::string_ref identity : auth_context->GetPeerIdentity()) {
      const std::string common_name(identity.data(), identity.size());
      if (std::find(allowed_client_common_names_.begin(),
                    allowed_client_common_names_.end(),
                    common_name) != allowed_client_common_names_.end()) {
        return ::grpc::Status::OK;
      }
    }
    return ::grpc::Status(::grpc::StatusCode::PERMISSION_DENIED,
                          "client certificate identity is not authorized");
  }

  ::grpc::Status ValidateRobotId(const std::string& robot_id) const {
    if (robot_id != robot_id_) {
      return ::grpc::Status(::grpc::StatusCode::NOT_FOUND,
                            "robot_id is not served by this instance");
    }
    return ::grpc::Status::OK;
  }

  static void PopulateLease(const ControlLease& lease,
                            robot::v1::ControlLease* response) {
    response->set_lease_id(lease.id);
    response->set_client_id(lease.client_id);
    const auto remaining = lease.expires_at - std::chrono::steady_clock::now();
    response->set_expires_in_ms(std::max<std::int64_t>(
        0, std::chrono::duration_cast<std::chrono::milliseconds>(remaining)
               .count()));
  }

  static StatusOr<CommandRequest> FromProtoCommand(
      const robot::v1::CommandRequest& source) {
    // protobuf oneof 到领域命令的唯一转换入口。这里做协议级检查（例如
    // 点动方向、超时），更深的状态/安全检查留给 CommandGuard。
    if (source.timeout_ms() < 0) {
      return Status(StatusCode::kInvalidArgument,
                    "command timeout must not be negative");
    }
    CommandRequest destination;
    destination.client_id = source.client_id();
    destination.lease_id = source.lease_id();
    destination.idempotency_key = source.idempotency_key();
    destination.timeout = std::chrono::milliseconds(
        source.timeout_ms() > 0 ? source.timeout_ms() : 30000);

    switch (source.command_case()) {
      case robot::v1::CommandRequest::kAcquireControl:
        destination.type = CommandType::kAcquireControl;
        break;
      case robot::v1::CommandRequest::kReleaseControl:
        destination.type = CommandType::kReleaseControl;
        break;
      case robot::v1::CommandRequest::kPowerOn:
        destination.type = CommandType::kPowerOn;
        break;
      case robot::v1::CommandRequest::kPowerOff:
        destination.type = CommandType::kPowerOff;
        break;
      case robot::v1::CommandRequest::kClearFault:
        destination.type = CommandType::kClearFault;
        break;
      case robot::v1::CommandRequest::kSetGlobalSpeed:
        destination.type = CommandType::kSetGlobalSpeed;
        destination.payload = SpeedPayload{source.set_global_speed().ratio()};
        break;
      case robot::v1::CommandRequest::kMoveJ: {
        if (source.move_j().target().degrees_size() == 0) {
          return Status(StatusCode::kInvalidArgument,
                        "MoveJ requires at least one joint value");
        }
        MoveJPayload payload;
        payload.target.position.degrees.reserve(source.move_j().target().degrees_size());
        for (int index = 0; index < source.move_j().target().degrees_size(); ++index) {
          payload.target.position.degrees.push_back(source.move_j().target().degrees(index));
        }
        payload.target.tool_name = source.move_j().tool_name();
        payload.target.work_object_name = source.move_j().work_object_name();
        payload.profile.speed_percent = source.move_j().speed_percent();
        payload.profile.blend = source.move_j().blend();
        destination.type = CommandType::kMoveJ;
        destination.payload = payload;
        break;
      }
      case robot::v1::CommandRequest::kMoveL: {
        MoveLPayload payload;
        payload.target.pose = FromProtoPose(source.move_l().target());
        payload.target.tool_name = source.move_l().tool_name();
        payload.target.work_object_name = source.move_l().work_object_name();
        payload.profile.speed_mm_per_second =
            source.move_l().speed_mm_per_second();
        payload.profile.blend_mm = source.move_l().blend_mm();
        destination.type = CommandType::kMoveL;
        destination.payload = payload;
        break;
      }
      case robot::v1::CommandRequest::kMoveC: {
        MoveCPayload payload;
        payload.target.via = FromProtoPose(source.move_c().via());
        payload.target.target = FromProtoPose(source.move_c().target());
        payload.target.tool_name = source.move_c().tool_name();
        payload.target.work_object_name = source.move_c().work_object_name();
        payload.profile.speed_mm_per_second =
            source.move_c().speed_mm_per_second();
        payload.profile.blend_mm = source.move_c().blend_mm();
        destination.type = CommandType::kMoveC;
        destination.payload = payload;
        break;
      }
      case robot::v1::CommandRequest::kHold:
        destination.type = CommandType::kHold;
        break;
      case robot::v1::CommandRequest::kResume:
        destination.type = CommandType::kResume;
        break;
      case robot::v1::CommandRequest::kStop:
        destination.type = CommandType::kStop;
        destination.payload = StopPayload{ToStopMode(source.stop().mode())};
        break;
      case robot::v1::CommandRequest::kJog:
        switch (source.jog().frame()) {
          case robot::v1::JOG_FRAME_UNSPECIFIED:
          case robot::v1::JOG_FRAME_JOINT:
          case robot::v1::JOG_FRAME_ROBOT:
          case robot::v1::JOG_FRAME_TOOL:
          case robot::v1::JOG_FRAME_USER:
          case robot::v1::JOG_FRAME_WORLD:
            break;
          default:
            return Status(StatusCode::kInvalidArgument, "jog frame is invalid");
        }
        if (source.jog().direction() != -1 && source.jog().direction() != 1) {
          return Status(StatusCode::kInvalidArgument,
                        "jog direction must be -1 or 1");
        }
        destination.type = CommandType::kJog;
        destination.payload =
            JogCommand{ToJogFrame(source.jog().frame()), source.jog().axis(),
                       source.jog().direction() < 0 ? JogDirection::kNegative
                                                    : JogDirection::kPositive,
                       source.jog().start()};
        break;
      case robot::v1::CommandRequest::kWriteDigitalOutput:
        destination.type = CommandType::kWriteDigitalOutput;
        destination.payload =
            DigitalOutputPayload{source.write_digital_output().index(),
                                 source.write_digital_output().value()};
        break;
      case robot::v1::CommandRequest::kLoadProgram:
        destination.type = CommandType::kLoadProgram;
        destination.payload = ProgramPayload{source.load_program().name()};
        break;
      case robot::v1::CommandRequest::kStartProgram:
        destination.type = CommandType::kStartProgram;
        break;
      case robot::v1::CommandRequest::kStopProgram:
        destination.type = CommandType::kStopProgram;
        break;
      case robot::v1::CommandRequest::COMMAND_NOT_SET:
        return Status(StatusCode::kInvalidArgument,
                      "command payload is required");
    }
    return destination;
  }

  static StopMode ToStopMode(robot::v1::StopMode mode) {
    switch (mode) {
      case robot::v1::STOP_MODE_QUICK:
        return StopMode::kQuick;
      case robot::v1::STOP_MODE_POWER_OFF_REQUEST:
        return StopMode::kPowerOffRequest;
      case robot::v1::STOP_MODE_UNSPECIFIED:
      case robot::v1::STOP_MODE_CONTROLLED:
        return StopMode::kControlled;
      default:
        return StopMode::kControlled;
    }
  }

  static JogFrame ToJogFrame(robot::v1::JogFrame frame) {
    // 未指定 frame 按关节点动处理，与领域模型的安全默认值保持一致。
    switch (frame) {
      case robot::v1::JOG_FRAME_ROBOT:
        return JogFrame::kRobot;
      case robot::v1::JOG_FRAME_TOOL:
        return JogFrame::kTool;
      case robot::v1::JOG_FRAME_USER:
        return JogFrame::kUser;
      case robot::v1::JOG_FRAME_WORLD:
        return JogFrame::kWorld;
      case robot::v1::JOG_FRAME_UNSPECIFIED:
      case robot::v1::JOG_FRAME_JOINT:
        return JogFrame::kJoint;
      default:
        return JogFrame::kJoint;
    }
  }

  const std::string robot_id_;
  RobotService* const service_;
  const std::vector<std::string> allowed_client_common_names_;
};

}  // namespace

Status RunGrpcServer(const GrpcServerOptions& options,
                     const std::string& robot_id, RobotService* service,
                     const volatile std::sig_atomic_t* stop_requested) {
  // 启动前校验配置，随后构造明文或双向 TLS 凭据并注册服务。
  if (options.listen_address.empty() || service == nullptr ||
      stop_requested == nullptr) {
    return Status(StatusCode::kInvalidArgument,
                  "gRPC listen address and executor are required");
  }
  if (options.allow_insecure_loopback &&
      !IsLoopbackListenAddress(options.listen_address)) {
    return Status(StatusCode::kInvalidArgument,
                  "insecure gRPC is permitted only on a loopback address");
  }
  if (!options.allow_insecure_loopback &&
      (options.trusted_client_ca_file.empty() ||
       options.server_certificate_chain_file.empty() ||
       options.server_private_key_file.empty() ||
       options.allowed_client_common_names.empty())) {
    return Status(StatusCode::kInvalidArgument,
                  "gRPC mTLS requires credentials and allowed identities");
  }

  std::shared_ptr<::grpc::ServerCredentials> credentials;
  if (options.allow_insecure_loopback) {
    credentials = ::grpc::InsecureServerCredentials();
  } else {
    const StatusOr<std::string> client_ca =
        ReadFile(options.trusted_client_ca_file);
    const StatusOr<std::string> certificate_chain =
        ReadFile(options.server_certificate_chain_file);
    const StatusOr<std::string> private_key =
        ReadFile(options.server_private_key_file);
    if (!client_ca.ok()) {
      return client_ca.status();
    }
    if (!certificate_chain.ok()) {
      return certificate_chain.status();
    }
    if (!private_key.ok()) {
      return private_key.status();
    }
    ::grpc::SslServerCredentialsOptions ssl_options;
    ssl_options.pem_root_certs = client_ca.value();
    ssl_options.pem_key_cert_pairs.push_back(
        {private_key.value(), certificate_chain.value()});
    ssl_options.client_certificate_request =
        GRPC_SSL_REQUEST_AND_REQUIRE_CLIENT_CERTIFICATE_AND_VERIFY;
    credentials = ::grpc::SslServerCredentials(ssl_options);
  }

  RobotControlEndpoint endpoint(robot_id, service,
                                options.allowed_client_common_names);
  ::grpc::ServerBuilder builder;
  builder.AddListeningPort(options.listen_address, credentials);
  builder.RegisterService(&endpoint);
  std::unique_ptr<::grpc::Server> server = builder.BuildAndStart();
  if (server == nullptr) {
    return Status(StatusCode::kUnavailable, "failed to start the gRPC server");
  }
  // gRPC 自身负责接收线程；当前线程只监视进程信号，收到停止请求后
  // 给正在处理的 RPC 最多 5 秒完成收尾，再等待服务器彻底退出。
  while (*stop_requested == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  server->Shutdown(std::chrono::system_clock::now() + std::chrono::seconds(5));
  server->Wait();
  return Status::Ok();
}

}  // namespace robot
