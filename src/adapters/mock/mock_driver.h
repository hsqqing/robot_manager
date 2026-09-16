// 仅由 robot_mock 目标提供的内存驱动，用于单元测试和无硬件开发。
#ifndef ROBOT_ADAPTERS_MOCK_MOCK_DRIVER_H_
#define ROBOT_ADAPTERS_MOCK_MOCK_DRIVER_H_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "robot/control/robot_driver.h"

namespace robot {

class MockDriver final : public IRobotDriver {
 public:
  explicit MockDriver(std::chrono::milliseconds motion_duration =
                          std::chrono::milliseconds(150));

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

  void SetEmergencyStop(bool active);
  void SetAlarm(bool active);
  void SetControllerMode(ControllerMode mode);

 private:
  Status CheckReadyLocked() const;
  void BeginMotionLocked();
  void RefreshMotionLocked();

  mutable std::mutex mutex_;
  std::chrono::milliseconds motion_duration_;
  std::chrono::steady_clock::time_point motion_ends_at_;
  RobotSnapshot state_;
  bool program_loaded_ = false;
  bool program_running_ = false;
  bool jog_active_ = false;
  std::vector<bool> digital_inputs_;
  std::vector<bool> digital_outputs_;
};

}  // namespace robot

#endif  // ROBOT_ADAPTERS_MOCK_MOCK_DRIVER_H_
