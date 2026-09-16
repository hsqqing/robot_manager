// 在命令进入驱动前执行租约、状态、速度和工作空间等安全检查。
#ifndef ROBOT_CONTROL_COMMAND_GUARD_H_
#define ROBOT_CONTROL_COMMAND_GUARD_H_

#include "robot/types/command.h"
#include "robot/types/status.h"
#include "robot/types/types.h"

namespace robot {

class CommandGuard {
 public:
  explicit CommandGuard(SafetyPolicy policy);

  Status Validate(const CommandRequest& request, const RobotSnapshot& snapshot,
                  bool lease_valid, bool jog_active) const;

 private:
  Status ValidateMoveJ(const MoveJPayload& payload) const;
  Status ValidateMoveL(const MoveLPayload& payload) const;
  Status ValidateMoveC(const MoveCPayload& payload) const;
  Status ValidateCommonMotion(const RobotSnapshot& snapshot,
                              bool lease_valid) const;

  SafetyPolicy policy_;
};

}  // namespace robot

#endif  // ROBOT_CONTROL_COMMAND_GUARD_H_
