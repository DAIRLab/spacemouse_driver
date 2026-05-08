#pragma once

#include <iostream>
#include <string>

#include <optional>

#include "drake/common/find_runfiles.h"

inline std::optional<std::string> FindRunfile(std::string resource_path) {
  auto main_runfile = drake::FindRunfile("_main/" + resource_path);
  auto external_runfile = drake::FindRunfile("spacemouse+/" + resource_path);
  if (main_runfile.abspath.empty() && external_runfile.abspath.empty()) {
    std::cerr << "Could not find runfile: " << resource_path << ". "
              << main_runfile.error << ", " << external_runfile.error;
    return std::nullopt;
  }
  std::string absolute_path =
      (main_runfile.abspath.empty() ? external_runfile.abspath
                                    : main_runfile.abspath);
  return std::make_optional(std::move(absolute_path));
}
