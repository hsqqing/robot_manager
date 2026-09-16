// 将 EFORT SDK 错误码转换为统一 Status，隔离厂商错误语义。
#include "efort_error_mapper.h"

#include <string>

#include "SdkConstDef.h"

namespace robot {
namespace {

std::string ErrorMessage(int vendor_code) {
  switch (vendor_code) {
    case ERROR_OK:
      return "operation succeeded";
    case ERROR_CONNECT_FAILED:
      return "connection failed";
    case ERROR_NO_CONNECTION:
      return "robot is not connected";
    case ERROR_CONNECTION_BROKEN:
      return "robot connection was lost";
    case ERROR_ACCESS_REJECTED:
      return "controller rejected API access";
    case ERROR_CUR_MODE_NOT_SUPPORT:
      return "controller mode does not support this operation";
    case ERROR_SERVO_ON_FAILED:
      return "servo power-on failed";
    case ERROR_POWER_OFF_FAILED:
      return "servo power-off failed";
    case ERROR_SET_MODE_FAILED:
      return "setting controller mode failed";
    case ERROR_SET_SPEED_RATIO_FAILED:
      return "setting speed ratio failed";
    case ERROR_START_PROGRAM_FAILED:
      return "starting controller program failed";
    case ERROR_RESET_PROGRAM_FAILED:
      return "resetting controller program failed";
    case ERROR_LOAD_PROGRAM_FAILED:
      return "loading controller program failed";
    case ERROR_PARA_INVALID:
      return "SDK parameter is invalid";
    case ERROR_STILL_RUNNING:
      return "controller is still running";
    case ERROR_SET_PARA_FAILED:
      return "setting controller parameter failed";
    case ERROR_GET_PARA_FAILED:
      return "reading controller parameter failed";
    case ERROR_MOVE_FAILED:
      return "controller rejected the motion command";
    case ERROR_ENABLE_API_CONTROL_FAILED:
      return "changing API control state failed";
    case ERROR_THIRD_OPERATION_TIME_OUT:
      return "SDK operation timed out";
    case ERROR_ROBOT_NOT_IN_POSITION:
      return "robot is not in the requested position";
    case ERROR_TARGET_INVALID:
      return "motion target is unreachable";
    case ERROR_MEMORY_LACK:
      return "SDK reported insufficient memory";
    case ERROR_VERSION_MISMATCH:
      return "SDK and controller versions do not match";
    default:
      return "EFORT SDK operation failed";
  }
}

}  // namespace

Status MapEfortError(int vendor_code, const std::string& operation) {
  if (vendor_code == ERROR_OK) {
    return Status::Ok();
  }

  StatusCode code = StatusCode::kInternal;
  switch (vendor_code) {
    case ERROR_CONNECT_FAILED:
    case ERROR_NO_CONNECTION:
    case ERROR_CONNECTION_BROKEN:
    case ERROR_THIRD_RECEIVE_NO_DATA:
    case ERROR_THIRD_RECEIVE_LOSS_DATA:
      code = StatusCode::kUnavailable;
      break;
    case ERROR_ACCESS_REJECTED:
    case ERROR_ENABLE_API_CONTROL_FAILED:
      code = StatusCode::kPermissionDenied;
      break;
    case ERROR_PARA_INVALID:
    case ERROR_TARGET_INVALID:
      code = StatusCode::kInvalidArgument;
      break;
    case ERROR_THIRD_OPERATION_TIME_OUT:
      code = StatusCode::kDeadlineExceeded;
      break;
    case ERROR_CUR_MODE_NOT_SUPPORT:
    case ERROR_SERVO_ON_FAILED:
    case ERROR_POWER_OFF_FAILED:
    case ERROR_STILL_RUNNING:
    case ERROR_ROBOT_NOT_IN_POSITION:
    case ERROR_VERSION_MISMATCH:
      code = StatusCode::kFailedPrecondition;
      break;
    case ERROR_MOVE_FAILED:
    case ERROR_START_PROGRAM_FAILED:
    case ERROR_RESET_PROGRAM_FAILED:
    case ERROR_LOAD_PROGRAM_FAILED:
      code = StatusCode::kControllerFault;
      break;
    default:
      code = vendor_code >= ERROR_THIRD_PARTY ? StatusCode::kInternal
                                              : StatusCode::kControllerFault;
      break;
  }

  return Status(code, operation + ": " + ErrorMessage(vendor_code),
                vendor_code);
}

}  // namespace robot
