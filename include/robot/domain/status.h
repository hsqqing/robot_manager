// 跨领域统一的错误码、错误信息和 StatusOr 返回类型。
#ifndef ROBOT_DOMAIN_STATUS_H_
#define ROBOT_DOMAIN_STATUS_H_

#include <optional>
#include <string>
#include <utility>

namespace robot {

enum class StatusCode {
  kOk = 0,
  kInvalidArgument,
  kFailedPrecondition,
  kPermissionDenied,
  kNotFound,
  kAlreadyExists,
  kResourceExhausted,
  kUnavailable,
  kDeadlineExceeded,
  kControllerFault,
  kInternal,
};

class Status {
 public:
  Status() = default;
  Status(StatusCode code, std::string message, int vendor_code = 0)
      : code_(code), message_(std::move(message)), vendor_code_(vendor_code) {}

  static Status Ok() { return Status(); }

  bool ok() const { return code_ == StatusCode::kOk; }
  StatusCode code() const { return code_; }
  const std::string& message() const { return message_; }
  int vendor_code() const { return vendor_code_; }

 private:
  StatusCode code_ = StatusCode::kOk;
  std::string message_;
  int vendor_code_ = 0;
};

template <typename T>
class StatusOr {
 public:
  StatusOr(Status status) : status_(std::move(status)) {}
  StatusOr(const T& value) : value_(value) {}
  StatusOr(T&& value) : value_(std::move(value)) {}

  bool ok() const { return value_.has_value(); }
  const Status& status() const { return status_; }

  const T& value() const& { return *value_; }
  T& value() & { return *value_; }
  T&& value() && { return std::move(*value_); }

 private:
  Status status_;
  std::optional<T> value_;
};

}  // namespace robot

#endif  // ROBOT_DOMAIN_STATUS_H_
