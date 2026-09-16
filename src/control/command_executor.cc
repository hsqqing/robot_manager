// 命令执行器实现：后台线程负责轮询状态和按序执行写操作。
#include "robot/control/command_executor.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "robot/control/state_machine.h"
#include "robot/control/command_guard.h"

namespace robot {
namespace {

bool IsMotionCommand(CommandType type) {
  return type == CommandType::kMoveJ || type == CommandType::kMoveL ||
         type == CommandType::kMoveC || type == CommandType::kStartProgram;
}

bool CanRunDuringMotion(CommandType type) {
  return type == CommandType::kStop || type == CommandType::kHold ||
         type == CommandType::kResume || type == CommandType::kPowerOff;
}

bool IsTerminal(CommandState state) {
  return state == CommandState::kSucceeded || state == CommandState::kFailed ||
         state == CommandState::kCancelled ||
         state == CommandState::kTimedOut || state == CommandState::kStopped;
}

std::string IdempotencyMapKey(const CommandRequest& request) {
  return request.client_id + "\n" + request.idempotency_key;
}

bool PosesEqual(const CartesianPose& left, const CartesianPose& right) {
  return left.x_mm == right.x_mm && left.y_mm == right.y_mm &&
         left.z_mm == right.z_mm && left.a_deg == right.a_deg &&
         left.b_deg == right.b_deg && left.c_deg == right.c_deg &&
         left.configuration == right.configuration &&
         left.joint_1_turn == right.joint_1_turn &&
         left.joint_4_turn == right.joint_4_turn &&
         left.joint_6_turn == right.joint_6_turn;
}

bool PayloadsEqual(const CommandRequest& left, const CommandRequest& right) {
  if (left.type != right.type || left.lease_id != right.lease_id ||
      left.timeout != right.timeout ||
      left.payload.index() != right.payload.index()) {
    return false;
  }
  if (std::holds_alternative<std::monostate>(left.payload)) {
    return true;
  }
  switch (left.type) {
    case CommandType::kSetGlobalSpeed:
      return std::get<SpeedPayload>(left.payload).ratio ==
             std::get<SpeedPayload>(right.payload).ratio;
    case CommandType::kMoveJ: {
      const MoveJPayload& a = std::get<MoveJPayload>(left.payload);
      const MoveJPayload& b = std::get<MoveJPayload>(right.payload);
      return a.target.position.degrees == b.target.position.degrees &&
             a.target.tool_name == b.target.tool_name &&
             a.target.work_object_name == b.target.work_object_name &&
             a.profile.speed_percent == b.profile.speed_percent &&
             a.profile.blend == b.profile.blend;
    }
    case CommandType::kMoveL: {
      const MoveLPayload& a = std::get<MoveLPayload>(left.payload);
      const MoveLPayload& b = std::get<MoveLPayload>(right.payload);
      return PosesEqual(a.target.pose, b.target.pose) &&
             a.target.tool_name == b.target.tool_name &&
             a.target.work_object_name == b.target.work_object_name &&
             a.profile.speed_mm_per_second == b.profile.speed_mm_per_second &&
             a.profile.blend_mm == b.profile.blend_mm;
    }
    case CommandType::kMoveC: {
      const MoveCPayload& a = std::get<MoveCPayload>(left.payload);
      const MoveCPayload& b = std::get<MoveCPayload>(right.payload);
      return PosesEqual(a.target.via, b.target.via) &&
             PosesEqual(a.target.target, b.target.target) &&
             a.target.tool_name == b.target.tool_name &&
             a.target.work_object_name == b.target.work_object_name &&
             a.profile.speed_mm_per_second == b.profile.speed_mm_per_second &&
             a.profile.blend_mm == b.profile.blend_mm;
    }
    case CommandType::kStop:
      return std::get<StopPayload>(left.payload).mode ==
             std::get<StopPayload>(right.payload).mode;
    case CommandType::kJog: {
      const JogCommand& a = std::get<JogCommand>(left.payload);
      const JogCommand& b = std::get<JogCommand>(right.payload);
      return a.frame == b.frame && a.axis == b.axis &&
             a.direction == b.direction && a.start == b.start;
    }
    case CommandType::kWriteDigitalOutput: {
      const DigitalOutputPayload& a =
          std::get<DigitalOutputPayload>(left.payload);
      const DigitalOutputPayload& b =
          std::get<DigitalOutputPayload>(right.payload);
      return a.index == b.index && a.value == b.value;
    }
    case CommandType::kLoadProgram:
      return std::get<ProgramPayload>(left.payload).name ==
             std::get<ProgramPayload>(right.payload).name;
    case CommandType::kAcquireControl:
    case CommandType::kReleaseControl:
    case CommandType::kPowerOn:
    case CommandType::kPowerOff:
    case CommandType::kClearFault:
    case CommandType::kHold:
    case CommandType::kResume:
    case CommandType::kStartProgram:
    case CommandType::kStopProgram:
      return true;
  }
  return false;
}

}  // namespace

class CommandExecutor::Impl {
 public:
  Impl(std::unique_ptr<IRobotDriver> driver, SafetyPolicy safety_policy,
       ExecutorOptions options)
      : driver_(std::move(driver)),
        guard_(std::move(safety_policy)),
        options_(options),
        lease_manager_(options.control_lease_ttl) {}

  ~Impl() { Shutdown(); }

  Status Start(const ConnectionOptions& connection) {
    // 启动顺序：校验执行器参数 -> 加锁连接驱动并读取初始状态 ->
    // 发布首个快照 -> 启动“命令执行线程”和“状态轮询线程”。
    if (driver_ == nullptr) {
      return Status(StatusCode::kInvalidArgument, "robot driver is required");
    }
    if (options_.state_poll_interval <= std::chrono::milliseconds::zero() ||
        options_.state_stale_after <= std::chrono::milliseconds::zero() ||
        options_.motion_start_grace < std::chrono::milliseconds::zero() ||
        options_.control_lease_ttl <= std::chrono::milliseconds::zero() ||
        options_.jog_heartbeat_timeout <= std::chrono::milliseconds::zero() ||
        options_.state_stale_after < options_.state_poll_interval ||
        options_.state_poll_interval > options_.jog_heartbeat_timeout ||
        options_.maximum_queue_size == 0 ||
        options_.maximum_command_history <= options_.maximum_queue_size) {
      return Status(StatusCode::kInvalidArgument,
                    "executor timing and queue options are invalid");
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (running_ || started_once_) {
        return Status(StatusCode::kAlreadyExists,
                      "command executor cannot be started again");
      }
    }

    Status connect_status;
    StatusOr<RobotSnapshot> initial_state(
        Status(StatusCode::kUnavailable, "state has not been read"));
    {
      std::lock_guard<std::mutex> driver_lock(driver_mutex_);
      connect_status = driver_->Connect(connection);
      if (connect_status.ok()) {
        initial_state = driver_->ReadState();
      }
    }
    if (!connect_status.ok()) {
      return connect_status;
    }
    if (!initial_state.ok()) {
      std::lock_guard<std::mutex> driver_lock(driver_mutex_);
      driver_->Disconnect();
      return initial_state.status();
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_state_ = initial_state.value();
      latest_state_.sequence = 1;
      latest_state_.lifecycle = DeriveLifecycleState(latest_state_);
      shutting_down_ = false;
      running_ = true;
      started_once_ = true;
    }
    try {
      worker_thread_ = std::thread(&Impl::WorkerLoop, this);
      poll_thread_ = std::thread(&Impl::PollLoop, this);
    } catch (const std::system_error& error) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        shutting_down_ = true;
        running_ = false;
      }
      condition_.notify_all();
      if (worker_thread_.joinable()) {
        worker_thread_.join();
      }
      if (poll_thread_.joinable()) {
        poll_thread_.join();
      }
      {
        std::lock_guard<std::mutex> driver_lock(driver_mutex_);
        driver_->Disconnect();
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        shutting_down_ = false;
        started_once_ = false;
      }
      return Status(
          StatusCode::kResourceExhausted,
          std::string("failed to start executor threads: ") + error.what());
    }
    return Status::Ok();
  }

  void Shutdown() {
    // 关闭时先请求受控停止，再 join 工作线程，最后释放控制权并断开 SDK。
    // 这样可以避免轮询线程仍在使用驱动时被提前销毁。
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_ && !worker_thread_.joinable() && !poll_thread_.joinable()) {
        return;
      }
      shutting_down_ = true;
      running_ = false;
    }
    condition_.notify_all();

    // Request a stop before joining workers. Waiting for the polling thread
    // first delays stop until its next SDK operation has returned.
    Status stop_status;
    {
      std::lock_guard<std::mutex> driver_lock(driver_mutex_);
      stop_status = driver_->Stop(StopMode::kControlled);
    }
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    if (poll_thread_.joinable()) {
      poll_thread_.join();
    }

    {
      std::lock_guard<std::mutex> driver_lock(driver_mutex_);
      driver_->ReleaseControl();
      driver_->Disconnect();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    CancelCommandsForShutdownLocked();
    if (active_motion_id_.has_value()) {
      auto active = commands_.find(*active_motion_id_);
      if (active != commands_.end() && !IsTerminal(active->second.state)) {
        active->second.state =
            stop_status.ok() ? CommandState::kStopped : CommandState::kFailed;
        active->second.result = stop_status;
        active->second.updated_at = std::chrono::steady_clock::now();
      }
      active_motion_id_.reset();
      active_motion_seen_ = false;
    }
    ClearActiveJogLocked();
    stop_requested_ = false;
  }

  StatusOr<ControlLease> AcquireControlLease(const std::string& client_id) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_ || shutting_down_) {
        return Status(StatusCode::kUnavailable,
                      "command executor is not running");
      }
    }
    return lease_manager_.Acquire(client_id);
  }

  StatusOr<ControlLease> RenewControlLease(const std::string& lease_id,
                                           const std::string& client_id) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_ || shutting_down_) {
        return Status(StatusCode::kUnavailable,
                      "command executor is not running");
      }
    }
    return lease_manager_.Renew(lease_id, client_id);
  }

  Status ReleaseControlLease(const std::string& lease_id,
                             const std::string& client_id) {
    const Status status = lease_manager_.Release(lease_id, client_id);
    if (!status.ok()) {
      return status;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      CancelQueuedCommandsForLeaseLocked(lease_id, client_id);
      if (running_ && (active_motion_id_.has_value() || jog_active_)) {
        RequestInternalStopLocked("control lease released");
      }
    }
    condition_.notify_all();
    return Status::Ok();
  }

  StatusOr<std::uint64_t> Submit(CommandRequest request) {
    // Submit 只创建命令记录并放入有界队列。幂等键保证网络重试不会重复
    // 执行同一逻辑命令；Stop 使用队头插入，确保停止请求优先处理。
    if (request.client_id.empty()) {
      return Status(StatusCode::kInvalidArgument, "client_id is required");
    }
    if (request.idempotency_key.empty()) {
      return Status(StatusCode::kInvalidArgument,
                    "idempotency_key is required");
    }
    if (request.timeout <= std::chrono::milliseconds::zero()) {
      return Status(StatusCode::kInvalidArgument,
                    "command timeout must be positive");
    }

    RobotSnapshot snapshot;
    bool jog_active = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_ || shutting_down_) {
        return Status(StatusCode::kUnavailable,
                      "command executor is not running");
      }
      const std::string idempotency_key = IdempotencyMapKey(request);
      const auto existing = idempotency_.find(idempotency_key);
      if (existing != idempotency_.end()) {
        const auto record = commands_.find(existing->second);
        if (record != commands_.end() &&
            PayloadsEqual(record->second.request, request)) {
          return existing->second;
        }
        return Status(StatusCode::kAlreadyExists,
                      "idempotency key was used by a different command");
      }
      snapshot = latest_state_;
      jog_active = jog_active_;
    }

    const bool lease_valid =
        lease_manager_.IsValid(request.lease_id, request.client_id);
    const Status validation =
        guard_.Validate(request, snapshot, lease_valid, jog_active);
    if (!validation.ok()) {
      return validation;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || shutting_down_) {
      return Status(StatusCode::kUnavailable,
                    "command executor is not running");
    }
    const std::string idempotency_key = IdempotencyMapKey(request);
    const auto existing = idempotency_.find(idempotency_key);
    if (existing != idempotency_.end()) {
      const auto record = commands_.find(existing->second);
      if (record != commands_.end() &&
          PayloadsEqual(record->second.request, request)) {
        return existing->second;
      }
      return Status(StatusCode::kAlreadyExists,
                    "idempotency key was used by a different command");
    }
    if (IsMotionCommand(request.type)) {
      const bool motion_pending = std::any_of(
          commands_.begin(), commands_.end(), [](const auto& command) {
            return IsMotionCommand(command.second.request.type) &&
                   !IsTerminal(command.second.state);
          });
      if (motion_pending) {
        return Status(StatusCode::kResourceExhausted,
                      "another motion command is active or queued");
      }
    }
    if (request.type == CommandType::kStartProgram &&
        loaded_approved_program_.empty()) {
      return Status(StatusCode::kFailedPrecondition,
                    "no approved program was loaded by this service");
    }
    if (queue_.size() >= options_.maximum_queue_size) {
      return Status(StatusCode::kResourceExhausted, "command queue is full");
    }
    PruneCommandHistoryLocked();
    if (commands_.size() >= options_.maximum_command_history) {
      return Status(StatusCode::kResourceExhausted,
                    "command history is full of active records");
    }

    CommandRecord record;
    record.id = next_command_id_++;
    record.request = std::move(request);
    record.state = CommandState::kQueued;
    record.created_at = std::chrono::steady_clock::now();
    record.updated_at = record.created_at;
    const std::uint64_t command_id = record.id;
    idempotency_[idempotency_key] = command_id;
    commands_.emplace(command_id, std::move(record));
    if (commands_.at(command_id).request.type == CommandType::kStop) {
      queue_.push_front(command_id);
    } else {
      queue_.push_back(command_id);
    }
    condition_.notify_all();
    return command_id;
  }

  StatusOr<CommandRecord> GetCommand(std::uint64_t command_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto iterator = commands_.find(command_id);
    if (iterator == commands_.end()) {
      return Status(StatusCode::kNotFound, "command was not found");
    }
    return iterator->second;
  }

  RobotSnapshot GetState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_state_;
  }

  StatusOr<std::vector<Alarm>> ReadAlarms() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_ || shutting_down_) {
        return Status(StatusCode::kUnavailable,
                      "command executor is not running");
      }
    }
    std::lock_guard<std::mutex> driver_lock(driver_mutex_);
    return driver_->ReadAlarms();
  }

  StatusOr<bool> ReadDigitalInput(std::uint32_t index) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_ || shutting_down_) {
        return Status(StatusCode::kUnavailable,
                      "command executor is not running");
      }
    }
    std::lock_guard<std::mutex> driver_lock(driver_mutex_);
    return driver_->ReadDigitalInput(index);
  }

  StatusOr<bool> ReadDigitalOutput(std::uint32_t index) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_ || shutting_down_) {
        return Status(StatusCode::kUnavailable,
                      "command executor is not running");
      }
    }
    std::lock_guard<std::mutex> driver_lock(driver_mutex_);
    return driver_->ReadDigitalOutput(index);
  }

 private:
  using QueueIterator = std::deque<std::uint64_t>::iterator;

  QueueIterator FindExecutableCommandLocked() {
    return std::find_if(
        queue_.begin(), queue_.end(), [this](std::uint64_t command_id) {
          const auto record = commands_.find(command_id);
          if (record == commands_.end()) {
            return true;
          }
          if (!active_motion_id_.has_value()) {
            return true;
          }
          if (CanRunDuringMotion(record->second.request.type)) {
            return true;
          }
          if (record->second.request.type != CommandType::kStopProgram) {
            return false;
          }
          const auto active = commands_.find(*active_motion_id_);
          return active != commands_.end() &&
                 active->second.request.type == CommandType::kStartProgram;
        });
  }

  void WorkerLoop() {
    // 唯一的 SDK 写入线程：从队列取命令，二次执行安全校验，再调用驱动。
    // 二次校验用于覆盖“入队后状态/租约发生变化”的竞态窗口。
    while (true) {
      CommandRecord record;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] {
          return shutting_down_ ||
                 FindExecutableCommandLocked() != queue_.end();
        });
        if (shutting_down_) {
          return;
        }
        const QueueIterator queue_item = FindExecutableCommandLocked();
        if (queue_item == queue_.end()) {
          continue;
        }
        const std::uint64_t command_id = *queue_item;
        queue_.erase(queue_item);
        auto command = commands_.find(command_id);
        if (command == commands_.end()) {
          continue;
        }
        command->second.state = CommandState::kRunning;
        command->second.updated_at = std::chrono::steady_clock::now();
        record = command->second;
      }

      RobotSnapshot snapshot;
      bool jog_active = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot = latest_state_;
        jog_active = jog_active_;
      }
      const bool lease_valid = lease_manager_.IsValid(record.request.lease_id,
                                                      record.request.client_id);
      Status result =
          guard_.Validate(record.request, snapshot, lease_valid, jog_active);
      if (result.ok() && record.request.type == CommandType::kStartProgram) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (loaded_approved_program_.empty()) {
          result = Status(StatusCode::kFailedPrecondition,
                          "no approved program was loaded by this service");
        }
      }
      if (result.ok()) {
        try {
          result = Execute(record.request);
        } catch (const std::exception& exception) {
          result = Status(
              StatusCode::kInternal,
              std::string("driver threw an exception: ") + exception.what());
        } catch (...) {
          result = Status(StatusCode::kInternal,
                          "driver threw an unknown exception");
        }
      }

      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto command = commands_.find(record.id);
        if (command == commands_.end()) {
          continue;
        }
        command->second.result = result;
        command->second.updated_at = std::chrono::steady_clock::now();
        if (!result.ok()) {
          command->second.state = CommandState::kFailed;
        } else if (IsMotionCommand(record.request.type)) {
          const bool superseded =
              active_motion_id_.has_value() && *active_motion_id_ != record.id;
          if (superseded) {
            result = Status(StatusCode::kFailedPrecondition,
                            "another motion became active during startup");
            command->second.result = result;
            command->second.state = CommandState::kFailed;
            RequestInternalStopLocked("conflicting motion startup");
            continue;
          }
          active_motion_id_ = record.id;
          active_motion_started_at_ = std::chrono::steady_clock::now();
          active_motion_seen_ = false;
          stop_requested_ = false;
          if (!lease_manager_.IsValid(record.request.lease_id,
                                      record.request.client_id)) {
            RequestInternalStopLocked(
                "control lease expired while motion was starting");
          }
        } else {
          command->second.state = CommandState::kSucceeded;
          if (record.request.type == CommandType::kLoadProgram) {
            const auto* program =
                std::get_if<ProgramPayload>(&record.request.payload);
            loaded_approved_program_ = program == nullptr ? "" : program->name;
          }
          if (record.request.type == CommandType::kJog) {
            const auto* jog = std::get_if<JogCommand>(&record.request.payload);
            if (jog != nullptr && jog->start) {
              jog_active_ = true;
              active_jog_client_id_ = record.request.client_id;
              active_jog_lease_id_ = record.request.lease_id;
              jog_deadline_ = std::chrono::steady_clock::now() +
                              options_.jog_heartbeat_timeout;
              if (!lease_manager_.IsValid(active_jog_lease_id_,
                                          active_jog_client_id_)) {
                RequestInternalStopLocked(
                    "control lease expired while jog was starting");
              }
            } else {
              ClearActiveJogLocked();
            }
          }
          if (record.request.type == CommandType::kStop ||
              record.request.type == CommandType::kPowerOff) {
            CompleteActiveMotionLocked(CommandState::kStopped);
            ClearActiveJogLocked();
            stop_requested_ = false;
          }
          if (record.request.type == CommandType::kStopProgram) {
            CompleteActiveProgramLocked();
            stop_requested_ = false;
          }
        }
      }
      condition_.notify_all();
    }
  }

  Status Execute(const CommandRequest& request) {
    // driver_mutex_ 将所有 SDK 调用串行化；上层永远只看到领域 Status，
    // 厂商错误码已在具体 IRobotDriver 适配器中转换。
    std::lock_guard<std::mutex> driver_lock(driver_mutex_);
    switch (request.type) {
      case CommandType::kAcquireControl:
        return driver_->AcquireControl();
      case CommandType::kReleaseControl:
        return driver_->ReleaseControl();
      case CommandType::kPowerOn:
        return driver_->PowerOn();
      case CommandType::kPowerOff:
        return driver_->PowerOff();
      case CommandType::kClearFault:
        return driver_->ClearFault();
      case CommandType::kSetGlobalSpeed:
        return ExecuteSetSpeed(request.payload);
      case CommandType::kMoveJ:
        return ExecuteMoveJ(request.payload);
      case CommandType::kMoveL:
        return ExecuteMoveL(request.payload);
      case CommandType::kMoveC:
        return ExecuteMoveC(request.payload);
      case CommandType::kHold:
        return driver_->Hold();
      case CommandType::kResume:
        return driver_->Resume();
      case CommandType::kStop:
        return ExecuteStop(request.payload);
      case CommandType::kJog:
        return ExecuteJog(request.payload);
      case CommandType::kWriteDigitalOutput:
        return ExecuteDigitalOutput(request.payload);
      case CommandType::kLoadProgram:
        return ExecuteLoadProgram(request.payload);
      case CommandType::kStartProgram:
        return driver_->StartProgram();
      case CommandType::kStopProgram:
        return driver_->StopProgram();
    }
    return Status(StatusCode::kInternal, "unknown command type");
  }

  Status ExecuteSetSpeed(const CommandPayload& payload) {
    const auto* speed = std::get_if<SpeedPayload>(&payload);
    return speed == nullptr ? Status(StatusCode::kInvalidArgument,
                                     "speed payload is missing")
                            : driver_->SetGlobalSpeed(speed->ratio);
  }

  Status ExecuteMoveJ(const CommandPayload& payload) {
    const auto* motion = std::get_if<MoveJPayload>(&payload);
    return motion == nullptr ? Status(StatusCode::kInvalidArgument,
                                      "MoveJ payload is missing")
                             : driver_->MoveJ(motion->target, motion->profile);
  }

  Status ExecuteMoveL(const CommandPayload& payload) {
    const auto* motion = std::get_if<MoveLPayload>(&payload);
    return motion == nullptr ? Status(StatusCode::kInvalidArgument,
                                      "MoveL payload is missing")
                             : driver_->MoveL(motion->target, motion->profile);
  }

  Status ExecuteMoveC(const CommandPayload& payload) {
    const auto* motion = std::get_if<MoveCPayload>(&payload);
    return motion == nullptr ? Status(StatusCode::kInvalidArgument,
                                      "MoveC payload is missing")
                             : driver_->MoveC(motion->target, motion->profile);
  }

  Status ExecuteStop(const CommandPayload& payload) {
    const auto* stop = std::get_if<StopPayload>(&payload);
    return driver_->Stop(stop == nullptr ? StopMode::kControlled : stop->mode);
  }

  Status ExecuteJog(const CommandPayload& payload) {
    const auto* jog = std::get_if<JogCommand>(&payload);
    return jog == nullptr
               ? Status(StatusCode::kInvalidArgument, "jog payload is missing")
               : driver_->Jog(*jog);
  }

  Status ExecuteDigitalOutput(const CommandPayload& payload) {
    const auto* output = std::get_if<DigitalOutputPayload>(&payload);
    return output == nullptr
               ? Status(StatusCode::kInvalidArgument,
                        "digital output payload is missing")
               : driver_->WriteDigitalOutput(output->index, output->value);
  }

  Status ExecuteLoadProgram(const CommandPayload& payload) {
    const auto* program = std::get_if<ProgramPayload>(&payload);
    return program == nullptr ? Status(StatusCode::kInvalidArgument,
                                       "program payload is missing")
                              : driver_->LoadProgram(program->name);
  }

  void PollLoop() {
    // 独立轮询线程周期性读取控制器状态，更新带序列号和时间戳的快照，
    // 同时监测运动超时、状态失联、租约过期和点动心跳超时。
    while (true) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (shutting_down_) {
          return;
        }
      }

      StatusOr<RobotSnapshot> state(
          Status(StatusCode::kUnavailable, "state polling failed"));
      {
        std::lock_guard<std::mutex> driver_lock(driver_mutex_);
        state = driver_->ReadState();
      }

      {
        std::lock_guard<std::mutex> lock(mutex_);
        bool update_active_motion = false;
        const auto state_poll_now = std::chrono::steady_clock::now();
        if (state.ok()) {
          const std::uint64_t next_sequence = latest_state_.sequence + 1;
          latest_state_ = state.value();
          latest_state_.sequence = next_sequence;
          latest_state_.lifecycle = DeriveLifecycleState(latest_state_);
          update_active_motion = true;
        } else if (latest_state_.observed_at ==
                       std::chrono::steady_clock::time_point{} ||
                   latest_state_.observed_at > state_poll_now ||
                   state_poll_now - latest_state_.observed_at >=
                       options_.state_stale_after) {
          if (latest_state_.lifecycle != RobotLifecycleState::kUnknown) {
            ++latest_state_.sequence;
          }
          latest_state_.lifecycle = RobotLifecycleState::kUnknown;
          update_active_motion = true;
        }
        if (update_active_motion) {
          UpdateActiveMotionLocked();
        }
        if (active_motion_id_.has_value() &&
            !ActiveMotionLeaseIsValidLocked()) {
          RequestInternalStopLocked("control lease expired");
        }
        if (jog_active_ && (std::chrono::steady_clock::now() >= jog_deadline_ ||
                            !lease_manager_.IsValid(active_jog_lease_id_,
                                                    active_jog_client_id_))) {
          RequestInternalStopLocked("jog heartbeat or control lease expired");
        }
      }
      condition_.notify_all();

      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait_for(lock, options_.state_poll_interval,
                          [this] { return shutting_down_; });
      if (shutting_down_) {
        return;
      }
    }
  }

  void UpdateActiveMotionLocked() {
    // 根据轮询到的 moving/program_running 判断运动命令何时真正完成；
    // 若状态进入故障、断开或超时，则排入内部 Stop，确保动作收敛。
    if (!active_motion_id_.has_value()) {
      return;
    }
    auto command = commands_.find(*active_motion_id_);
    if (command == commands_.end()) {
      active_motion_id_.reset();
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = now - active_motion_started_at_;
    if (elapsed > command->second.request.timeout) {
      if (!IsTerminal(command->second.state)) {
        command->second.state = CommandState::kTimedOut;
        command->second.result =
            Status(StatusCode::kDeadlineExceeded, "motion command timed out");
        command->second.updated_at = now;
      }
      RequestInternalStopLocked("motion command timed out");
      return;
    }
    if (latest_state_.lifecycle == RobotLifecycleState::kUnknown ||
        latest_state_.lifecycle == RobotLifecycleState::kDisconnected ||
        latest_state_.lifecycle == RobotLifecycleState::kFault ||
        latest_state_.lifecycle == RobotLifecycleState::kEmergencyStop) {
      if (!IsTerminal(command->second.state)) {
        const bool unavailable =
            latest_state_.lifecycle == RobotLifecycleState::kUnknown ||
            latest_state_.lifecycle == RobotLifecycleState::kDisconnected;
        command->second.state = CommandState::kFailed;
        command->second.result =
            Status(unavailable ? StatusCode::kUnavailable
                               : StatusCode::kControllerFault,
                   "robot state became unsafe while motion was active");
        command->second.updated_at = now;
      }
      RequestInternalStopLocked("robot state became unsafe");
      active_motion_id_.reset();
      active_motion_seen_ = false;
      return;
    }
    if (latest_state_.paused) {
      return;
    }
    if (command->second.request.type == CommandType::kStartProgram) {
      if (latest_state_.program_running) {
        active_motion_seen_ = true;
        return;
      }
      if (active_motion_seen_ || elapsed >= options_.motion_start_grace) {
        if (!IsTerminal(command->second.state)) {
          command->second.state = CommandState::kSucceeded;
          command->second.result = Status::Ok();
          command->second.updated_at = now;
        }
        active_motion_id_.reset();
        active_motion_seen_ = false;
        stop_requested_ = false;
      }
      return;
    }
    if (latest_state_.moving) {
      active_motion_seen_ = true;
      return;
    }
    if (active_motion_seen_ || elapsed >= options_.motion_start_grace) {
      if (!IsTerminal(command->second.state)) {
        command->second.state = CommandState::kSucceeded;
        command->second.result = Status::Ok();
        command->second.updated_at = now;
      }
      active_motion_id_.reset();
      active_motion_seen_ = false;
      stop_requested_ = false;
    }
  }

  void RequestInternalStopLocked(const std::string& reason) {
    if (stop_requested_) {
      return;
    }
    CommandRecord stop;
    stop.id = next_command_id_++;
    stop.request.type = CommandType::kStop;
    stop.request.payload = StopPayload{};
    stop.request.client_id = "robot-manager";
    stop.request.idempotency_key = "internal-stop-" + std::to_string(stop.id);
    stop.state = CommandState::kQueued;
    stop.result = Status(StatusCode::kFailedPrecondition, reason);
    stop.created_at = std::chrono::steady_clock::now();
    stop.updated_at = stop.created_at;
    const std::uint64_t stop_id = stop.id;
    commands_.emplace(stop_id, std::move(stop));
    queue_.push_front(stop_id);
    stop_requested_ = true;
  }

  void CompleteActiveMotionLocked(CommandState state) {
    if (!active_motion_id_.has_value()) {
      return;
    }
    auto active = commands_.find(*active_motion_id_);
    if (active != commands_.end() && !IsTerminal(active->second.state)) {
      active->second.state = state;
      active->second.updated_at = std::chrono::steady_clock::now();
    }
    active_motion_id_.reset();
    active_motion_seen_ = false;
  }

  void CancelQueuedCommandsForLeaseLocked(const std::string& lease_id,
                                          const std::string& client_id) {
    auto queued = queue_.begin();
    while (queued != queue_.end()) {
      const auto command = commands_.find(*queued);
      if (command == commands_.end()) {
        queued = queue_.erase(queued);
        continue;
      }
      const CommandRequest& request = command->second.request;
      if (request.type == CommandType::kStop || request.lease_id != lease_id ||
          request.client_id != client_id) {
        ++queued;
        continue;
      }
      command->second.state = CommandState::kCancelled;
      command->second.result =
          Status(StatusCode::kPermissionDenied, "control lease released");
      command->second.updated_at = std::chrono::steady_clock::now();
      queued = queue_.erase(queued);
    }
  }

  void CancelCommandsForShutdownLocked() {
    const auto now = std::chrono::steady_clock::now();
    for (const std::uint64_t command_id : queue_) {
      const auto command = commands_.find(command_id);
      if (command == commands_.end() || IsTerminal(command->second.state)) {
        continue;
      }
      command->second.state = CommandState::kCancelled;
      command->second.result =
          Status(StatusCode::kUnavailable, "command executor shut down");
      command->second.updated_at = now;
    }
    queue_.clear();
  }

  void PruneCommandHistoryLocked() {
    while (commands_.size() >= options_.maximum_command_history) {
      auto oldest = commands_.end();
      for (auto command = commands_.begin(); command != commands_.end();
           ++command) {
        if (!IsTerminal(command->second.state) ||
            (oldest != commands_.end() &&
             command->second.updated_at >= oldest->second.updated_at)) {
          continue;
        }
        oldest = command;
      }
      if (oldest == commands_.end()) {
        return;
      }
      idempotency_.erase(IdempotencyMapKey(oldest->second.request));
      commands_.erase(oldest);
    }
  }

  void CompleteActiveProgramLocked() {
    if (!active_motion_id_.has_value()) {
      return;
    }
    const auto active = commands_.find(*active_motion_id_);
    if (active == commands_.end() ||
        active->second.request.type != CommandType::kStartProgram) {
      return;
    }
    CompleteActiveMotionLocked(CommandState::kStopped);
  }

  bool ActiveMotionLeaseIsValidLocked() const {
    if (!active_motion_id_.has_value()) {
      return true;
    }
    const auto active = commands_.find(*active_motion_id_);
    return active != commands_.end() &&
           lease_manager_.IsValid(active->second.request.lease_id,
                                  active->second.request.client_id);
  }

  void ClearActiveJogLocked() {
    jog_active_ = false;
    active_jog_client_id_.clear();
    active_jog_lease_id_.clear();
  }

  std::unique_ptr<IRobotDriver> driver_;
  CommandGuard guard_;
  ExecutorOptions options_;
  ControlLeaseManager lease_manager_;

  mutable std::mutex mutex_;
  std::mutex driver_mutex_;
  std::condition_variable condition_;
  std::thread worker_thread_;
  std::thread poll_thread_;
  bool running_ = false;
  bool started_once_ = false;
  bool shutting_down_ = false;
  bool stop_requested_ = false;
  bool jog_active_ = false;
  std::string active_jog_client_id_;
  std::string active_jog_lease_id_;
  std::chrono::steady_clock::time_point jog_deadline_;

  RobotSnapshot latest_state_;
  std::deque<std::uint64_t> queue_;
  std::unordered_map<std::uint64_t, CommandRecord> commands_;
  std::unordered_map<std::string, std::uint64_t> idempotency_;
  std::uint64_t next_command_id_ = 1;

  std::optional<std::uint64_t> active_motion_id_;
  std::chrono::steady_clock::time_point active_motion_started_at_;
  bool active_motion_seen_ = false;
  std::string loaded_approved_program_;
};

CommandExecutor::CommandExecutor(std::unique_ptr<IRobotDriver> driver,
                                 SafetyPolicy safety_policy,
                                 ExecutorOptions options)
    : impl_(std::make_unique<Impl>(std::move(driver), std::move(safety_policy),
                                   options)) {}

CommandExecutor::~CommandExecutor() = default;

Status CommandExecutor::Start(const ConnectionOptions& connection) {
  return impl_->Start(connection);
}

void CommandExecutor::Shutdown() { impl_->Shutdown(); }

StatusOr<ControlLease> CommandExecutor::AcquireControlLease(
    const std::string& client_id) {
  return impl_->AcquireControlLease(client_id);
}

StatusOr<ControlLease> CommandExecutor::RenewControlLease(
    const std::string& lease_id, const std::string& client_id) {
  return impl_->RenewControlLease(lease_id, client_id);
}

Status CommandExecutor::ReleaseControlLease(const std::string& lease_id,
                                            const std::string& client_id) {
  return impl_->ReleaseControlLease(lease_id, client_id);
}

StatusOr<std::uint64_t> CommandExecutor::Submit(CommandRequest request) {
  return impl_->Submit(std::move(request));
}

StatusOr<CommandRecord> CommandExecutor::GetCommand(
    std::uint64_t command_id) const {
  return impl_->GetCommand(command_id);
}

RobotSnapshot CommandExecutor::GetState() const { return impl_->GetState(); }

StatusOr<std::vector<Alarm>> CommandExecutor::ReadAlarms() {
  return impl_->ReadAlarms();
}

StatusOr<bool> CommandExecutor::ReadDigitalInput(std::uint32_t index) {
  return impl_->ReadDigitalInput(index);
}

StatusOr<bool> CommandExecutor::ReadDigitalOutput(std::uint32_t index) {
  return impl_->ReadDigitalOutput(index);
}

}  // namespace robot
