// EFORT 适配器的公共声明；仅由 robot_efort 目标提供，SDK 细节保持私有。
#ifndef ROBOT_ADAPTERS_EFORT_EFORT_DRIVER_H_
#define ROBOT_ADAPTERS_EFORT_EFORT_DRIVER_H_

#include <memory>

#include "robot/control/robot_driver.h"

namespace robot {

class EfortDriver final : public IRobotDriver {
 public:
  EfortDriver();
  ~EfortDriver() override;

  EfortDriver(const EfortDriver&) = delete;
  EfortDriver& operator=(const EfortDriver&) = delete;

  Status Connect(const ConnectionOptions& options) override;
  Status Disconnect() override;
  Status AcquireControl() override;
  Status ReleaseControl() override;
  Status PowerOn() override;
  Status PowerOff() override;
  Status ClearFault() override;
  Status SetGlobalSpeed(unsigned int ratio) override;
  Status MoveJ(const JointTarget& target,
               const JointMotionProfile& profile) override;
  Status MoveL(const CartesianTarget& target,
               const CartesianMotionProfile& profile) override;
  Status MoveC(const CircularTarget& target,
               const CartesianMotionProfile& profile) override;
  Status Hold() override;
  Status Resume() override;
  Status Stop(StopMode mode) override;
  Status Jog(const JogCommand& command) override;
  Status LoadProgram(const std::string& name) override;
  Status StartProgram() override;
  Status StopProgram() override;
  StatusOr<RobotSnapshot> ReadState() override;
  StatusOr<std::vector<Alarm>> ReadAlarms() override;
  StatusOr<bool> ReadDigitalInput(std::uint32_t index) override;
  StatusOr<bool> ReadDigitalOutput(std::uint32_t index) override;
  Status WriteDigitalOutput(std::uint32_t index, bool value) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot

#endif  // ROBOT_ADAPTERS_EFORT_EFORT_DRIVER_H_
