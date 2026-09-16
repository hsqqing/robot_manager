#ifndef ROBOT_TRANSPORT_GRPC_OPTIONS_H_
#define ROBOT_TRANSPORT_GRPC_OPTIONS_H_

#include <string>
#include <vector>

#include "robot/types/status.h"

namespace robot {

struct GrpcServerOptions {
  std::string listen_address;
  bool allow_insecure_loopback = false;
  std::string trusted_client_ca_file;
  std::string server_certificate_chain_file;
  std::string server_private_key_file;
  std::vector<std::string> allowed_client_common_names;
};

StatusOr<GrpcServerOptions> LoadGrpcServerOptions(const std::string& path);

}  // namespace robot

#endif  // ROBOT_TRANSPORT_GRPC_OPTIONS_H_
