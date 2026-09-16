#include "robot/config/config_loader.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#define RYML_SINGLE_HDR_DEFINE_NOW
#include <ryml_all.hpp>


namespace robot {
namespace {

struct YamlDocument {
  std::map<std::string, std::string> scalars;
  std::map<std::string, std::vector<std::string>> lists;
  std::set<std::string> mappings;
};

Status ConfigError(const std::string& message) {
  return Status(StatusCode::kInvalidArgument, "configuration: " + message);
}

std::string YamlString(ryml::csubstr value) {
  return value.len == 0 ? std::string{} : std::string(value.str, value.len);
}

Status ReadMapping(ryml::ConstNodeRef node, const std::string& prefix,
                   YamlDocument& document) {
  if (!node.is_map()) {
    return ConfigError("expected a mapping at '" + prefix + "'");
  }
  for (const auto child : node.children()) {
    const std::string key = YamlString(child.key());
    if (key.empty() || key.find('.') != std::string::npos ||
        child.has_anchor() || child.is_ref() ||
        child.has_key_tag() || child.has_val_tag()) {
      return ConfigError("unsupported key or YAML reference at '" + prefix + "'");
    }
    const std::string path = prefix.empty() ? key : prefix + "." + key;
    if (document.scalars.count(path) || document.lists.count(path) ||
        document.mappings.count(path)) {
      return ConfigError("duplicate key '" + path + "'");
    }
    if (child.is_map()) {
      document.mappings.insert(path);
      const Status status = ReadMapping(child, path, document);
      if (!status.ok()) return status;
    } else if (child.is_seq()) {
      auto& values = document.lists[path];
      for (const auto item : child.children()) {
        if (!item.is_val() || item.val_is_null() || item.has_anchor() ||
            item.is_ref() || item.has_val_tag()) {
          return ConfigError("setting '" + path + "' must be a scalar list");
        }
        values.push_back(YamlString(item.val()));
      }
    } else if (child.has_val() && !child.val_is_null()) {
      document.scalars.emplace(path, YamlString(child.val()));
    } else {
      return ConfigError("setting '" + path + "' must not be null");
    }
  }
  return Status::Ok();
}

StatusOr<YamlDocument> ParseYaml(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    return Status(StatusCode::kNotFound, "cannot open configuration: " + path);
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  if (input.bad()) return ConfigError("cannot read " + path);
  const std::string source = buffer.str();

  // 使用局部回调，让语法错误返回 Status，避免默认错误处理终止进程。
  ryml::Callbacks callbacks;
  callbacks.m_error_basic = [](ryml::csubstr msg, const ryml::ErrorDataBasic&, void*) {
    throw std::runtime_error(YamlString(msg));
  };
  callbacks.m_error_parse = [](ryml::csubstr msg, const ryml::ErrorDataParse&, void*) {
    throw std::runtime_error(YamlString(msg));
  };
  callbacks.m_error_visit = [](ryml::csubstr msg, const ryml::ErrorDataVisit&, void*) {
    throw std::runtime_error(YamlString(msg));
  };
  try {
    ryml::Tree tree(callbacks);
    ryml::parse_in_arena(ryml::to_csubstr(path), ryml::to_csubstr(source), &tree);
    YamlDocument document;
    const Status status = ReadMapping(tree.crootref(), "", document);
    if (!status.ok()) return status;
    return document;
  } catch (const std::runtime_error& error) {
    return ConfigError(error.what());
  }
}

Status CheckKnownKeys(const YamlDocument& document) {
  const std::set<std::string> known_scalars = {
      "schema_version", "service.instance_id", "service.grpc_listen",
      "service.state_poll_ms", "service.state_stale_after_ms",
      "service.command_queue_capacity", "service.command_history_capacity",
      "robot.id", "robot.driver", "robot.address", "robot.model",
      "robot.axis_count",
      "robot.verify_sdk_version", "robot.sdk_debug_logging",
      "control_lease.ttl_ms", "control_lease.jog_heartbeat_timeout_ms",
      "limits.motion_enabled", "limits.program_execution_enabled",
      "limits.maximum_joint_speed_percent",
      "limits.maximum_linear_speed_mm_per_second",
      "limits.cartesian_workspace.frame", "limits.cartesian_workspace.x_mm",
      "limits.cartesian_workspace.y_mm", "limits.cartesian_workspace.z_mm",
      "security.grpc.allow_insecure_loopback",
      "security.grpc.client_ca_file",
      "security.grpc.server_certificate_chain_file",
      "security.grpc.server_private_key_file"};
  const std::set<std::string> known_lists = {
      "limits.joint_limits_deg", "limits.writable_digital_outputs",
      "limits.approved_programs", "security.grpc.allowed_client_common_names"};
  const std::set<std::string> known_mappings = {
      "service", "robot", "control_lease", "limits",
      "limits.cartesian_workspace", "security", "security.grpc"};
  for (const auto& item : document.scalars) {
    if (known_scalars.count(item.first) == 0) {
      return ConfigError("unknown setting '" + item.first + "'");
    }
  }
  for (const auto& item : document.lists) {
    if (known_lists.count(item.first) == 0) {
      return ConfigError("unknown list setting '" + item.first + "'");
    }
  }
  for (const std::string& mapping : document.mappings) {
    if (known_mappings.count(mapping) == 0 &&
        known_lists.count(mapping) == 0) {
      return ConfigError("unknown section '" + mapping + "'");
    }
  }
  return Status::Ok();
}

StatusOr<std::string> RequiredScalar(const YamlDocument& document,
                                     const std::string& key) {
  const auto iterator = document.scalars.find(key);
  if (iterator == document.scalars.end() || iterator->second.empty()) {
    return ConfigError("missing required setting '" + key + "'");
  }
  return iterator->second;
}

StatusOr<bool> ParseBool(const YamlDocument& document, const std::string& key) {
  const StatusOr<std::string> value = RequiredScalar(document, key);
  if (!value.ok()) {
    return value.status();
  }
  if (value.value() == "true") {
    return true;
  }
  if (value.value() == "false") {
    return false;
  }
  return ConfigError("setting '" + key + "' must be true or false");
}

StatusOr<int> ParseInt(const YamlDocument& document, const std::string& key) {
  const StatusOr<std::string> value = RequiredScalar(document, key);
  if (!value.ok()) {
    return value.status();
  }
  char* end = nullptr;
  errno = 0;
  const long parsed = std::strtol(value.value().c_str(), &end, 10);
  if (errno != 0 || end == value.value().c_str() || *end != '\0' ||
      parsed < std::numeric_limits<int>::min() ||
      parsed > std::numeric_limits<int>::max()) {
    return ConfigError("setting '" + key + "' must be an integer");
  }
  return static_cast<int>(parsed);
}

StatusOr<int> ParseIntScalar(const std::string& value,
                             const std::string& key) {
  YamlDocument document;
  document.scalars.emplace(key, value);
  return ParseInt(document, key);
}

StatusOr<double> ParseDouble(const std::string& value, const std::string& key) {
  char* end = nullptr;
  errno = 0;
  const double parsed = std::strtod(value.c_str(), &end);
  if (errno != 0 || end == value.c_str() || *end != '\0' ||
      !std::isfinite(parsed)) {
    return ConfigError("setting '" + key + "' must be a finite number");
  }
  return parsed;
}

StatusOr<JointLimit> ParseRange(const std::string& value,
                                const std::string& key) {
  const std::size_t separator = value.find(':');
  if (separator == std::string::npos ||
      value.find(':', separator + 1) != std::string::npos) {
    return ConfigError("setting '" + key + "' must use min:max");
  }
  const StatusOr<double> minimum = ParseDouble(value.substr(0, separator), key);
  const StatusOr<double> maximum = ParseDouble(value.substr(separator + 1), key);
  if (!minimum.ok()) {
    return minimum.status();
  }
  if (!maximum.ok()) {
    return maximum.status();
  }
  if (minimum.value() >= maximum.value()) {
    return ConfigError("setting '" + key + "' must have min < max");
  }
  return JointLimit{minimum.value(), maximum.value()};
}

StatusOr<std::vector<std::string>> OptionalList(const YamlDocument& document,
                                                const std::string& key) {
  const auto iterator = document.lists.find(key);
  if (iterator == document.lists.end()) {
    if (document.mappings.count(key) != 0 ||
        document.scalars.count(key) != 0) {
      return ConfigError("setting '" + key + "' must be a scalar list");
    }
    return std::vector<std::string>{};
  }
  if (iterator->second.empty()) {
    return ConfigError("list '" + key + "' must not be empty");
  }
  return iterator->second;
}

bool IsLoopbackListenAddress(const std::string& address) {
  return address.rfind("127.", 0) == 0 ||
         address.rfind("localhost:", 0) == 0 ||
         address.rfind("[::1]:", 0) == 0 ||
         address.rfind("unix:", 0) == 0;
}

}  // namespace

StatusOr<ServiceConfig> LoadServiceConfig(const std::string& path) {
  const StatusOr<YamlDocument> document = ParseYaml(path);
  if (!document.ok()) {
    return document.status();
  }
  const Status known_keys = CheckKnownKeys(document.value());
  if (!known_keys.ok()) {
    return known_keys;
  }

  ServiceConfig config;
  const StatusOr<int> schema_version = ParseInt(document.value(), "schema_version");
  if (!schema_version.ok()) {
    return schema_version.status();
  }
  if (schema_version.value() != 1) {
    return ConfigError("unsupported schema_version");
  }
  const StatusOr<std::string> instance_id =
      RequiredScalar(document.value(), "service.instance_id");
  const StatusOr<std::string> grpc_listen =
      RequiredScalar(document.value(), "service.grpc_listen");
  const StatusOr<std::string> robot_id = RequiredScalar(document.value(), "robot.id");
  const StatusOr<std::string> driver_name =
      RequiredScalar(document.value(), "robot.driver");
  const StatusOr<std::string> address = RequiredScalar(document.value(), "robot.address");
  const StatusOr<std::string> model_name = RequiredScalar(document.value(), "robot.model");
  const auto axis_count_setting = document.value().scalars.find("robot.axis_count");
  const StatusOr<bool> verify_sdk = ParseBool(document.value(), "robot.verify_sdk_version");
  const StatusOr<bool> sdk_debug = ParseBool(document.value(), "robot.sdk_debug_logging");
  const StatusOr<int> state_poll = ParseInt(document.value(), "service.state_poll_ms");
  const StatusOr<int> state_stale = ParseInt(document.value(), "service.state_stale_after_ms");
  const StatusOr<int> queue_size = ParseInt(document.value(), "service.command_queue_capacity");
  const StatusOr<int> history_size = ParseInt(document.value(), "service.command_history_capacity");
  const StatusOr<int> lease_ttl = ParseInt(document.value(), "control_lease.ttl_ms");
  const StatusOr<int> jog_timeout = ParseInt(document.value(), "control_lease.jog_heartbeat_timeout_ms");
  const StatusOr<bool> motion_enabled = ParseBool(document.value(), "limits.motion_enabled");
  const StatusOr<bool> programs_enabled = ParseBool(document.value(), "limits.program_execution_enabled");
  const StatusOr<int> maximum_joint_speed =
      ParseInt(document.value(), "limits.maximum_joint_speed_percent");
  const StatusOr<int> maximum_linear_speed =
      ParseInt(document.value(), "limits.maximum_linear_speed_mm_per_second");
  const StatusOr<bool> insecure_loopback =
      ParseBool(document.value(), "security.grpc.allow_insecure_loopback");
  if (!instance_id.ok() || !grpc_listen.ok() || !robot_id.ok() ||
      !driver_name.ok() || !address.ok() || !model_name.ok() || !verify_sdk.ok() ||
      !sdk_debug.ok() || !state_poll.ok() || !state_stale.ok() ||
      !queue_size.ok() || !history_size.ok() || !lease_ttl.ok() ||
      !jog_timeout.ok() || !motion_enabled.ok() || !programs_enabled.ok() ||
      !maximum_joint_speed.ok() || !maximum_linear_speed.ok() ||
      !insecure_loopback.ok()) {
    const std::array<Status, 19> statuses = {
        instance_id.status(), grpc_listen.status(), robot_id.status(),
        driver_name.status(), address.status(), model_name.status(), verify_sdk.status(),
        sdk_debug.status(), state_poll.status(), state_stale.status(),
        queue_size.status(), history_size.status(), lease_ttl.status(),
        jog_timeout.status(), motion_enabled.status(), programs_enabled.status(),
        maximum_joint_speed.status(), maximum_linear_speed.status(),
        insecure_loopback.status()};
    for (const Status& status : statuses) {
      if (!status.ok()) {
        return status;
      }
    }
  }

  if (state_poll.value() < 1 || state_stale.value() < state_poll.value() ||
      queue_size.value() < 1 || history_size.value() < 1 ||
      lease_ttl.value() < 1 || jog_timeout.value() < state_poll.value() ||
      maximum_joint_speed.value() < 1 || maximum_joint_speed.value() > 100 ||
      maximum_linear_speed.value() < 1) {
    return ConfigError("timing, queue, or speed limits are out of range");
  }

  config.instance_id = instance_id.value();
  config.driver_name = driver_name.value();
  config.connection.robot_id = robot_id.value();
  config.connection.address = address.value();
  config.connection.expected_model = model_name.value();
  if (axis_count_setting != document.value().scalars.end()) {
    const StatusOr<int> axis_count =
        ParseIntScalar(axis_count_setting->second, "robot.axis_count");
    if (!axis_count.ok() || axis_count.value() < 1) {
      return ConfigError("robot.axis_count must be a positive integer");
    }
    config.connection.expected_axis_count =
        static_cast<std::size_t>(axis_count.value());
  }
  config.connection.verify_sdk_version = verify_sdk.value();
  config.connection.sdk_debug_logging = sdk_debug.value();
  config.executor_options.state_poll_interval =
      std::chrono::milliseconds(state_poll.value());
  config.executor_options.state_stale_after =
      std::chrono::milliseconds(state_stale.value());
  config.executor_options.control_lease_ttl =
      std::chrono::milliseconds(lease_ttl.value());
  config.executor_options.jog_heartbeat_timeout =
      std::chrono::milliseconds(jog_timeout.value());
  config.executor_options.maximum_queue_size =
      static_cast<std::size_t>(queue_size.value());
  config.executor_options.maximum_command_history =
      static_cast<std::size_t>(history_size.value());
  config.safety_policy.motion_enabled = motion_enabled.value();
  config.safety_policy.program_execution_enabled = programs_enabled.value();
  config.safety_policy.maximum_joint_speed_percent = maximum_joint_speed.value();
  config.safety_policy.maximum_linear_speed_mm_per_second =
      maximum_linear_speed.value();

  const StatusOr<std::vector<std::string>> joint_limits =
      OptionalList(document.value(), "limits.joint_limits_deg");
  if (!joint_limits.ok()) {
    return joint_limits.status();
  }
  if (!joint_limits.value().empty()) {
    config.safety_policy.joint_limits.resize(joint_limits.value().size());
    for (std::size_t index = 0; index < joint_limits.value().size(); ++index) {
      const StatusOr<JointLimit> range =
          ParseRange(joint_limits.value()[index], "limits.joint_limits_deg");
      if (!range.ok()) {
        return range.status();
      }
      config.safety_policy.joint_limits[index] = range.value();
    }
    config.safety_policy.joint_limits_verified = true;
  }
  if (config.connection.expected_axis_count != 0 &&
      config.safety_policy.joint_limits_verified &&
      config.safety_policy.joint_limits.size() !=
          config.connection.expected_axis_count) {
    return ConfigError(
        "limits.joint_limits_deg count must match robot.axis_count");
  }
  if (config.safety_policy.motion_enabled &&
      !config.safety_policy.joint_limits_verified) {
    return ConfigError("motion requires verified joint limits");
  }

  const auto frame = document.value().scalars.find("limits.cartesian_workspace.frame");
  const auto x_range = document.value().scalars.find("limits.cartesian_workspace.x_mm");
  const auto y_range = document.value().scalars.find("limits.cartesian_workspace.y_mm");
  const auto z_range = document.value().scalars.find("limits.cartesian_workspace.z_mm");
  const bool has_workspace = frame != document.value().scalars.end() ||
      x_range != document.value().scalars.end() || y_range != document.value().scalars.end() ||
      z_range != document.value().scalars.end();
  if (has_workspace) {
    if (frame == document.value().scalars.end() || frame->second.empty() ||
        x_range == document.value().scalars.end() ||
        y_range == document.value().scalars.end() || z_range == document.value().scalars.end()) {
      return ConfigError("cartesian workspace requires frame and x_mm/y_mm/z_mm ranges");
    }
    const StatusOr<JointLimit> x = ParseRange(x_range->second, "limits.cartesian_workspace.x_mm");
    const StatusOr<JointLimit> y = ParseRange(y_range->second, "limits.cartesian_workspace.y_mm");
    const StatusOr<JointLimit> z = ParseRange(z_range->second, "limits.cartesian_workspace.z_mm");
    if (!x.ok()) return x.status();
    if (!y.ok()) return y.status();
    if (!z.ok()) return z.status();
    config.safety_policy.cartesian_workspace = {x.value().minimum_deg, x.value().maximum_deg,
        y.value().minimum_deg, y.value().maximum_deg, z.value().minimum_deg, z.value().maximum_deg};
    config.safety_policy.cartesian_workspace_frame = frame->second;
    config.safety_policy.cartesian_workspace_verified = true;
  }

  const StatusOr<std::vector<std::string>> outputs =
      OptionalList(document.value(), "limits.writable_digital_outputs");
  if (!outputs.ok()) return outputs.status();
  for (const std::string& output : outputs.value()) {
    const StatusOr<int> index = ParseIntScalar(output, "writable output");
    if (!index.ok() || index.value() < 0) {
      return ConfigError("limits.writable_digital_outputs contains an invalid index");
    }
    config.safety_policy.writable_digital_outputs.push_back(
        static_cast<std::uint32_t>(index.value()));
  }
  const StatusOr<std::vector<std::string>> programs =
      OptionalList(document.value(), "limits.approved_programs");
  if (!programs.ok()) return programs.status();
  config.safety_policy.approved_programs = programs.value();
  if (config.safety_policy.program_execution_enabled && programs.value().empty()) {
    return ConfigError("program execution requires approved programs");
  }

  config.grpc.listen_address = grpc_listen.value();
  config.grpc.allow_insecure_loopback = insecure_loopback.value();
  const auto copy_optional = [&document, &config](const std::string& key,
                                                   std::string* destination) {
    const auto iterator = document.value().scalars.find(key);
    if (iterator != document.value().scalars.end()) *destination = iterator->second;
  };
  copy_optional("security.grpc.client_ca_file", &config.grpc.trusted_client_ca_file);
  copy_optional("security.grpc.server_certificate_chain_file",
                &config.grpc.server_certificate_chain_file);
  copy_optional("security.grpc.server_private_key_file",
                &config.grpc.server_private_key_file);
  const StatusOr<std::vector<std::string>> common_names = OptionalList(
      document.value(), "security.grpc.allowed_client_common_names");
  if (!common_names.ok()) return common_names.status();
  config.grpc.allowed_client_common_names = common_names.value();
  if (config.grpc.allow_insecure_loopback) {
    if (!IsLoopbackListenAddress(config.grpc.listen_address)) {
      return ConfigError("insecure gRPC is permitted only on a loopback address");
    }
  } else if (config.grpc.trusted_client_ca_file.empty() ||
             config.grpc.server_certificate_chain_file.empty() ||
             config.grpc.server_private_key_file.empty() ||
             config.grpc.allowed_client_common_names.empty()) {
    return ConfigError("gRPC mTLS requires CA, certificate, private key, and allowed client names");
  }
  return config;
}

}  // namespace robot
