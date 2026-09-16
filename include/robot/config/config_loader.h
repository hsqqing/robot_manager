// 服务 YAML 配置结构，以及配置加载与完整性校验接口。
#ifndef ROBOT_CONFIG_CONFIG_LOADER_H_
#define ROBOT_CONFIG_CONFIG_LOADER_H_

#include <string>

#include "robot/control/command_executor.h"
#include "robot/types/status.h"
#include "robot/types/types.h"

namespace robot {

struct ServiceConfig {
  std::string instance_id;
  std::string driver_name;
  ConnectionOptions connection;
  SafetyPolicy safety_policy;
  ExecutorOptions executor_options;
};

// Loads and validates the transport-independent service settings. Application
// extensions under `transport` are validated by the selected application.
StatusOr<ServiceConfig> LoadServiceConfig(const std::string& path);

}  // namespace robot

#endif  // ROBOT_CONFIG_CONFIG_LOADER_H_
