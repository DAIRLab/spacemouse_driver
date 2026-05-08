#pragma once

#include <functional>
#include <string>

#include <lcm/lcm-cpp.hpp>

#include "configs/spacemouse_settings.h"

struct SpacemouseStatePublisherConfig {
  std::string lcm_url;
  std::string state_lcm_channel;
  int state_publish_rate{};
  SpacemouseSettings settings;
};

using ButtonPressHandler = std::function<void(int, lcm::LCM&)>;

int RunSpacemouseStatePublisher(const SpacemouseStatePublisherConfig& config,
                                ButtonPressHandler handle_button_press);
