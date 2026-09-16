// 控制租约模型及其并发安全的管理器接口。
#ifndef ROBOT_SERVICE_CONTROL_LEASE_H_
#define ROBOT_SERVICE_CONTROL_LEASE_H_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include "robot/domain/status.h"

namespace robot {

struct ControlLease {
  std::string id;
  std::string client_id;
  std::chrono::steady_clock::time_point expires_at;
};

class ControlLeaseManager {
 public:
  explicit ControlLeaseManager(std::chrono::milliseconds lease_ttl);

  StatusOr<ControlLease> Acquire(const std::string& client_id);
  StatusOr<ControlLease> Renew(const std::string& lease_id,
                               const std::string& client_id);
  Status Release(const std::string& lease_id, const std::string& client_id);
  bool IsValid(const std::string& lease_id, const std::string& client_id) const;

 private:
  bool IsActiveLocked(std::chrono::steady_clock::time_point now) const;

  const std::chrono::milliseconds lease_ttl_;
  mutable std::mutex mutex_;
  ControlLease active_;
  bool has_active_ = false;
  std::uint64_t next_id_ = 1;
};

}  // namespace robot

#endif  // ROBOT_SERVICE_CONTROL_LEASE_H_
