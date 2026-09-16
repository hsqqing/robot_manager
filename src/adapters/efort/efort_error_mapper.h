#ifndef ROBOT_ADAPTERS_EFORT_EFORT_ERROR_MAPPER_H_
#define ROBOT_ADAPTERS_EFORT_EFORT_ERROR_MAPPER_H_

#include <string>

#include "robot/domain/status.h"

namespace robot {

Status MapEfortError(int vendor_code, const std::string& operation);

}  // namespace robot

#endif  // ROBOT_ADAPTERS_EFORT_EFORT_ERROR_MAPPER_H_
