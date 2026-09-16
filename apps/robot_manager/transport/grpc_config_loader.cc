#include "robot_manager/transport/grpc_options.h"

#include <set>
#include <string>

#include "robot/config/config_document.h"

namespace robot {
namespace {

Status ConfigError(const std::string& message) {
  return Status(StatusCode::kInvalidArgument,
                "gRPC configuration: " + message);
}

bool IsLoopbackListenAddress(const std::string& address) {
  return address.rfind("127.", 0) == 0 ||
         address.rfind("localhost:", 0) == 0 ||
         address.rfind("[::1]:", 0) == 0 ||
         address.rfind("unix:", 0) == 0;
}

StatusOr<std::string> Required(const ConfigDocument& document,
                               const std::string& key) {
  const auto value = document.scalars.find(key);
  if (value == document.scalars.end() || value->second.empty()) {
    return ConfigError("missing required setting '" + key + "'");
  }
  return value->second;
}

void CopyOptional(const ConfigDocument& document, const std::string& key,
                  std::string* destination) {
  const auto value = document.scalars.find(key);
  if (value != document.scalars.end()) {
    *destination = value->second;
  }
}

}  // namespace

StatusOr<GrpcServerOptions> LoadGrpcServerOptions(const std::string& path) {
  const StatusOr<ConfigDocument> document = LoadConfigDocument(path);
  if (!document.ok()) {
    return document.status();
  }

  constexpr char kPrefix[] = "transport.grpc.";
  const std::set<std::string> known_scalars = {
      std::string(kPrefix) + "listen",
      std::string(kPrefix) + "allow_insecure_loopback",
      std::string(kPrefix) + "client_ca_file",
      std::string(kPrefix) + "server_certificate_chain_file",
      std::string(kPrefix) + "server_private_key_file"};
  const std::string common_names_key =
      std::string(kPrefix) + "allowed_client_common_names";
  for (const auto& scalar : document.value().scalars) {
    if (scalar.first.rfind(kPrefix, 0) == 0 &&
        known_scalars.count(scalar.first) == 0) {
      return ConfigError("unknown setting '" + scalar.first + "'");
    }
  }
  for (const auto& list : document.value().lists) {
    if (list.first.rfind(kPrefix, 0) == 0 && list.first != common_names_key) {
      return ConfigError("unknown list setting '" + list.first + "'");
    }
  }
  for (const std::string& mapping : document.value().mappings) {
    if (mapping.rfind("transport.grpc", 0) == 0 &&
        mapping != "transport.grpc") {
      return ConfigError("unknown section '" + mapping + "'");
    }
  }

  GrpcServerOptions options;
  const StatusOr<std::string> listen =
      Required(document.value(), std::string(kPrefix) + "listen");
  const StatusOr<std::string> insecure = Required(
      document.value(), std::string(kPrefix) + "allow_insecure_loopback");
  if (!listen.ok()) return listen.status();
  if (!insecure.ok()) return insecure.status();
  if (insecure.value() != "true" && insecure.value() != "false") {
    return ConfigError("allow_insecure_loopback must be true or false");
  }
  options.listen_address = listen.value();
  options.allow_insecure_loopback = insecure.value() == "true";
  CopyOptional(document.value(), std::string(kPrefix) + "client_ca_file",
               &options.trusted_client_ca_file);
  CopyOptional(document.value(),
               std::string(kPrefix) + "server_certificate_chain_file",
               &options.server_certificate_chain_file);
  CopyOptional(document.value(), std::string(kPrefix) + "server_private_key_file",
               &options.server_private_key_file);
  const auto names = document.value().lists.find(common_names_key);
  if (names != document.value().lists.end()) {
    options.allowed_client_common_names = names->second;
  }

  if (options.allow_insecure_loopback) {
    if (!IsLoopbackListenAddress(options.listen_address)) {
      return ConfigError("insecure transport is permitted only on loopback");
    }
  } else if (options.trusted_client_ca_file.empty() ||
             options.server_certificate_chain_file.empty() ||
             options.server_private_key_file.empty() ||
             options.allowed_client_common_names.empty()) {
    return ConfigError(
        "mTLS requires CA, certificate, private key, and allowed client names");
  }
  return options;
}

}  // namespace robot
