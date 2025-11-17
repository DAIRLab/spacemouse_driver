#include "iostream"
#include "string"

#include "optional"

#include "drake/common/find_runfiles.h"

std::optional<std::string> FindRunfile(std::string resource_path) {
  // Set default solver options
  auto main_runfile = drake::FindRunfile("_main/" + resource_path);
  auto external_runfile = drake::FindRunfile("spacemouse+/" + resource_path);
  if (main_runfile.abspath.empty() && external_runfile.abspath.empty()) {
    std::cerr << "Could not find the default solver options YAML file. "
              << main_runfile.error << ", " << external_runfile.error;
    return std::nullopt;
  }
  std::string absolute_path =
      (main_runfile.abspath.empty() ? external_runfile.abspath
                                    : main_runfile.abspath);
  return std::make_optional(std::move(absolute_path));
}