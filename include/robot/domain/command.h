// 领域命令、载荷和命令执行记录；不依赖具体机器人厂商 SDK。
#ifndef ROBOT_DOMAIN_COMMAND_H_
#define ROBOT_DOMAIN_COMMAND_H_

#include <chrono>
#include <cstdint>
#include <string>
#include <variant>

#include "robot/domain/status.h"
#include "robot/domain/types.h"

namespace robot {

enum class CommandType {
  kAcquireControl,
  kReleaseControl,
  kPowerOn,
  kPowerOff,
  kClearFault,
  kSetGlobalSpeed,
  kMoveJ,
  kMoveL,
  kMoveC,
  kHold,
  kResume,
  kStop,
  kJog,
  kWriteDigitalOutput,
  kLoadProgram,
  kStartProgram,
  kStopProgram,
};

enum class CommandState {
  kQueued,
  kRunning,
  kSucceeded,
  kFailed,
  kCancelled,
  kTimedOut,
  kStopped,
};

struct MoveJPayload {
  JointTarget target;
  JointMotionProfile profile;
};

struct MoveLPayload {
  CartesianTarget target;
  CartesianMotionProfile profile;
};

struct MoveCPayload {
  CircularTarget target;
  CartesianMotionProfile profile;
};

struct StopPayload {
  StopMode mode = StopMode::kControlled;
};

struct SpeedPayload {
  unsigned int ratio = 10;
};

struct DigitalOutputPayload {
  std::uint32_t index = 0;
  bool value = false;
};

struct ProgramPayload {
  std::string name;
};

using CommandPayload =
    std::variant<std::monostate, MoveJPayload, MoveLPayload, MoveCPayload,
                 StopPayload, SpeedPayload, JogCommand, DigitalOutputPayload,
                 ProgramPayload>;

struct CommandRequest {
  // idempotency_key 以 client_id 为命名空间：同键重试返回已有 command_id，
  // 但使用同键提交不同内容会被拒绝，防止网络重试造成重复动作。
  CommandType type = CommandType::kStop;
  CommandPayload payload;
  std::string client_id;
  std::string lease_id;
  std::string idempotency_key;
  std::chrono::milliseconds timeout{30000};
};

struct CommandRecord {
  std::uint64_t id = 0;
  CommandRequest request;
  CommandState state = CommandState::kQueued;
  Status result;
  std::chrono::steady_clock::time_point created_at;
  std::chrono::steady_clock::time_point updated_at;
};

}  // namespace robot

#endif  // ROBOT_DOMAIN_COMMAND_H_
