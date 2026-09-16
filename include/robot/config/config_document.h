// Transport-neutral flattened representation of a deployment YAML document.
#ifndef ROBOT_CONFIG_CONFIG_DOCUMENT_H_
#define ROBOT_CONFIG_CONFIG_DOCUMENT_H_

#include <map>
#include <set>
#include <string>
#include <vector>

#include "robot/types/status.h"

namespace robot {

struct ConfigDocument {
  std::map<std::string, std::string> scalars;
  std::map<std::string, std::vector<std::string>> lists;
  std::set<std::string> mappings;
};

StatusOr<ConfigDocument> LoadConfigDocument(const std::string& path);

}  // namespace robot

#endif  // ROBOT_CONFIG_CONFIG_DOCUMENT_H_
