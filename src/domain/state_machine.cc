// 纯函数式状态推导逻辑，便于独立测试且不产生外部副作用。
#include "robot/domain/state_machine.h"

namespace robot {

RobotLifecycleState DeriveLifecycleState(const RobotSnapshot& snapshot) {
  // 优先级从最危险状态到可执行状态：急停/断连/故障不能被 moving 等普通
  // 标志覆盖，使客户端始终得到保守的生命周期结论。
  if (snapshot.emergency_stop) {
    return RobotLifecycleState::kEmergencyStop;
  }
  if (!snapshot.connected) {
    return RobotLifecycleState::kDisconnected;
  }
  if (snapshot.alarm_active) {
    return RobotLifecycleState::kFault;
  }
  if (snapshot.paused) {
    return RobotLifecycleState::kPaused;
  }
  if (snapshot.moving || snapshot.program_running) {
    return RobotLifecycleState::kExecuting;
  }
  if (snapshot.api_control && snapshot.servo_on &&
      snapshot.controller_mode == ControllerMode::kAutomatic) {
    return RobotLifecycleState::kReady;
  }
  return RobotLifecycleState::kStandby;
}

const char* LifecycleStateName(RobotLifecycleState state) {
  switch (state) {
    case RobotLifecycleState::kDisconnected:
      return "DISCONNECTED";
    case RobotLifecycleState::kConnecting:
      return "CONNECTING";
    case RobotLifecycleState::kStandby:
      return "STANDBY";
    case RobotLifecycleState::kReady:
      return "READY";
    case RobotLifecycleState::kExecuting:
      return "EXECUTING";
    case RobotLifecycleState::kPaused:
      return "PAUSED";
    case RobotLifecycleState::kFault:
      return "FAULT";
    case RobotLifecycleState::kEmergencyStop:
      return "ESTOP";
    case RobotLifecycleState::kUnknown:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

}  // namespace robot
