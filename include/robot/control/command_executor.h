// 应用层执行门面：串行化机器人命令并维护状态、租约与生命周期。
#ifndef ROBOT_CONTROL_COMMAND_EXECUTOR_H_
#define ROBOT_CONTROL_COMMAND_EXECUTOR_H_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "robot/control/control_lease.h"
#include "robot/control/robot_driver.h"
#include "robot/control/robot_service.h"
#include "robot/types/command.h"
#include "robot/types/status.h"
#include "robot/types/types.h"

namespace robot {

struct ExecutorOptions {
  // 状态轮询、运动启动宽限、租约及点动看门狗共同决定服务的故障收敛时间。
  std::chrono::milliseconds state_poll_interval{50};
  std::chrono::milliseconds state_stale_after{500};
  std::chrono::milliseconds motion_start_grace{500};
  std::chrono::milliseconds control_lease_ttl{3000};
  std::chrono::milliseconds jog_heartbeat_timeout{300};
  std::size_t maximum_queue_size = 16;
  std::size_t maximum_command_history = 4096;
};

class CommandExecutor : public RobotService {
  // 应用层门面：持有驱动、串行执行写命令、缓存状态，并管理租约和命令生命周期。
 public:
  CommandExecutor(std::unique_ptr<IRobotDriver> driver,
                  SafetyPolicy safety_policy, ExecutorOptions options = {});
  ~CommandExecutor();

  CommandExecutor(const CommandExecutor&) = delete;
  CommandExecutor& operator=(const CommandExecutor&) = delete;

  Status Start(const ConnectionOptions& connection);
  void Shutdown();

  StatusOr<ControlLease> AcquireControlLease(
      const std::string& client_id) override;
  StatusOr<ControlLease> RenewControlLease(
      const std::string& lease_id, const std::string& client_id) override;
  Status ReleaseControlLease(const std::string& lease_id,
                             const std::string& client_id) override;

  StatusOr<std::uint64_t> Submit(CommandRequest request) override;
  StatusOr<CommandRecord> GetCommand(std::uint64_t command_id) const override;
  RobotSnapshot GetState() const override;
  StatusOr<std::vector<Alarm>> ReadAlarms() override;
  StatusOr<bool> ReadDigitalInput(std::uint32_t index) override;
  StatusOr<bool> ReadDigitalOutput(std::uint32_t index) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot

#endif  // ROBOT_CONTROL_COMMAND_EXECUTOR_H_
