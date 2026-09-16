#ifndef ROBOT_SERVICE_ROBOT_SERVICE_H_
#define ROBOT_SERVICE_ROBOT_SERVICE_H_
#include <cstdint>
#include <string>
#include <vector>
#include "robot/service/control_lease.h"
#include "robot/domain/command.h"
#include "robot/domain/types.h"
namespace robot {
class RobotService {
 public:
  virtual ~RobotService() = default;
  virtual StatusOr<ControlLease> AcquireControlLease(const std::string&) = 0;
  virtual StatusOr<ControlLease> RenewControlLease(const std::string&, const std::string&) = 0;
  virtual Status ReleaseControlLease(const std::string&, const std::string&) = 0;
  virtual StatusOr<std::uint64_t> Submit(CommandRequest) = 0;
  virtual StatusOr<CommandRecord> GetCommand(std::uint64_t) const = 0;
  virtual RobotSnapshot GetState() const = 0;
  virtual StatusOr<std::vector<Alarm>> ReadAlarms() = 0;
  virtual StatusOr<bool> ReadDigitalInput(std::uint32_t) = 0;
  virtual StatusOr<bool> ReadDigitalOutput(std::uint32_t) = 0;
};
}  // namespace robot

#endif  // ROBOT_SERVICE_ROBOT_SERVICE_H_
