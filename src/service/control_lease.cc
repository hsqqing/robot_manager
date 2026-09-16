// 控制租约的分配、续期、释放和过期回收实现。
#include "robot/service/control_lease.h"

#include <chrono>
#include <string>

namespace robot {

ControlLeaseManager::ControlLeaseManager(std::chrono::milliseconds lease_ttl)
    : lease_ttl_(lease_ttl) {}

StatusOr<ControlLease> ControlLeaseManager::Acquire(
    const std::string& client_id) {
  if (client_id.empty()) {
    return Status(StatusCode::kInvalidArgument, "client_id is required");
  }

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (IsActiveLocked(now)) {
    return Status(StatusCode::kAlreadyExists,
                  "another client holds the control lease");
  }

  const auto timestamp = now.time_since_epoch().count();
  active_.id =
      "lease-" + std::to_string(timestamp) + "-" + std::to_string(next_id_++);
  active_.client_id = client_id;
  active_.expires_at = now + lease_ttl_;
  has_active_ = true;
  return active_;
}

StatusOr<ControlLease> ControlLeaseManager::Renew(
    const std::string& lease_id, const std::string& client_id) {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!IsActiveLocked(now) || active_.id != lease_id ||
      active_.client_id != client_id) {
    return Status(StatusCode::kPermissionDenied,
                  "control lease is invalid or expired");
  }
  active_.expires_at = now + lease_ttl_;
  return active_;
}

Status ControlLeaseManager::Release(const std::string& lease_id,
                                    const std::string& client_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_active_ || active_.id != lease_id ||
      active_.client_id != client_id) {
    return Status(StatusCode::kPermissionDenied,
                  "control lease does not belong to this client");
  }
  has_active_ = false;
  active_ = ControlLease{};
  return Status::Ok();
}

bool ControlLeaseManager::IsValid(const std::string& lease_id,
                                  const std::string& client_id) const {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  return IsActiveLocked(now) && active_.id == lease_id &&
         active_.client_id == client_id;
}

bool ControlLeaseManager::IsActiveLocked(
    std::chrono::steady_clock::time_point now) const {
  return has_active_ && now < active_.expires_at;
}

}  // namespace robot
