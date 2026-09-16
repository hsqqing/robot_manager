// 服务 YAML 配置结构，以及配置加载与完整性校验接口。
#ifndef ROBOT_CONFIG_CONFIG_LOADER_H_
#define ROBOT_CONFIG_CONFIG_LOADER_H_

#include <string>
#include <vector>

#include "robot/control/command_executor.h"
#include "robot/types/status.h"
#include "robot/types/types.h"
namespace robot {

// Transport-neutral settings consumed by the executable's selected transport.
// Keeping this value type in the public config API prevents robot_core from
// depending on an application-specific gRPC header.
struct GrpcTransportConfig {
  std::string listen_address;
  bool allow_insecure_loopback = false;
  std::string trusted_client_ca_file;
  std::string server_certificate_chain_file;
  std::string server_private_key_file;
  std::vector<std::string> allowed_client_common_names;
};

struct ServiceConfig {
  std::string instance_id;
  std::string driver_name;
  ConnectionOptions connection;
  SafetyPolicy safety_policy;
  ExecutorOptions executor_options;
  GrpcTransportConfig grpc;
};

// Loads the supported YAML subset and validates the complete service schema.
// Invalid, missing, duplicate, or unknown settings cause a non-OK result.
StatusOr<ServiceConfig> LoadServiceConfig(const std::string& path);

}  // namespace robot

#endif  // ROBOT_CONFIG_CONFIG_LOADER_H_
