// 机器人驱动抽象接口；应用层只依赖此协议，不直接依赖厂商 SDK。
#ifndef ROBOT_DRIVER_ROBOT_DRIVER_H_
#define ROBOT_DRIVER_ROBOT_DRIVER_H_

#include <cstdint>
#include <string>
#include <vector>

#include "robot/domain/status.h"
#include "robot/domain/types.h"

namespace robot {

class IRobotDriver {
 public:
  virtual ~IRobotDriver() = default;

  virtual Status Connect(const ConnectionOptions& options) = 0;
  virtual Status Disconnect() = 0;
  virtual Status AcquireControl() = 0;
  virtual Status ReleaseControl() = 0;
  virtual Status PowerOn() = 0;
  virtual Status PowerOff() = 0;
  virtual Status ClearFault() = 0;
  virtual Status SetGlobalSpeed(unsigned int ratio) = 0;

  virtual Status MoveJ(const JointTarget& target,
                       const JointMotionProfile& profile) = 0;
  virtual Status MoveL(const CartesianTarget& target,
                       const CartesianMotionProfile& profile) = 0;
  virtual Status MoveC(const CircularTarget& target,
                       const CartesianMotionProfile& profile) = 0;
  virtual Status Hold() = 0;
  virtual Status Resume() = 0;
  virtual Status Stop(StopMode mode) = 0;
  virtual Status Jog(const JogCommand& command) = 0;

  virtual Status LoadProgram(const std::string& name) = 0;
  virtual Status StartProgram() = 0;
  virtual Status StopProgram() = 0;

  virtual StatusOr<RobotSnapshot> ReadState() = 0;
  virtual StatusOr<std::vector<Alarm>> ReadAlarms() = 0;
  virtual StatusOr<bool> ReadDigitalInput(std::uint32_t index) = 0;
  virtual StatusOr<bool> ReadDigitalOutput(std::uint32_t index) = 0;
  virtual Status WriteDigitalOutput(std::uint32_t index, bool value) = 0;
};

}  // namespace robot

#endif  // ROBOT_DRIVER_ROBOT_DRIVER_H_
