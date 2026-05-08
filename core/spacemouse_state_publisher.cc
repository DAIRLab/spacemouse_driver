#include "core/spacemouse_state_publisher.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <spnav.h>

#include "spacemouse/lcmt_spacemouse_state.hpp"

namespace {

std::atomic<bool> running{true};

void SignalHandler(int signum) {
  std::cout << "\nInterrupt signal (" << signum << ") received.\n";
  running = false;
}

void ZeroStaticAxes(const spnav_event& event,
                    const SpacemouseSettings& settings, double mapped_linear[3],
                    double mapped_angular[3]) {
  const int* motion_values[] = {&event.motion.x, &event.motion.y,
                                &event.motion.z};
  for (int i = 0; i < 3; ++i) {
    if (std::abs(*motion_values[settings.linear_axis[i]]) <
        settings.static_trans_deadband) {
      mapped_linear[i] = 0;
    }
  }

  const int* rot_motion_values[] = {&event.motion.rx, &event.motion.ry,
                                    &event.motion.rz};
  for (int i = 0; i < 3; ++i) {
    if (std::abs(*rot_motion_values[settings.angular_axis[i]]) <
        settings.static_rot_deadband) {
      mapped_angular[i] = 0;
    }
  }
}

void ClampInvalidAxes(double values[3]) {
  for (int i = 0; i < 3; i++) {
    if (values[i] < -1.0 || values[i] > 1.0) {
      values[i] = 0.0;
    }
  }
}

bool IsValidAxisMapping(const std::vector<int>& axes,
                        const std::vector<double>& signs,
                        const std::string& name) {
  if (axes.size() != 3 || signs.size() != 3) {
    std::cerr << name << " axis and sign settings must both have 3 entries."
              << std::endl;
    return false;
  }

  for (int axis : axes) {
    if (axis < 0 || axis > 2) {
      std::cerr << name << " axis entries must be 0, 1, or 2." << std::endl;
      return false;
    }
  }
  return true;
}

void ApplyAxisMapping(const double raw_values[3], const std::vector<int>& axes,
                      const std::vector<double>& signs,
                      double mapped_values[3]) {
  for (int i = 0; i < 3; ++i) {
    mapped_values[i] = signs[i] * raw_values[axes[i]];
  }
}

}  // namespace

int RunSpacemouseStatePublisher(const SpacemouseStatePublisherConfig& config,
                                ButtonPressHandler handle_button_press) {
  running = true;
  std::signal(SIGINT, SignalHandler);

  if (!IsValidAxisMapping(config.settings.linear_axis,
                          config.settings.linear_sign, "linear") ||
      !IsValidAxisMapping(config.settings.angular_axis,
                          config.settings.angular_sign, "angular")) {
    return 1;
  }

  const auto state_period =
      std::chrono::duration<double>(1.0 / config.state_publish_rate);

  if (spnav_open() == -1) {
    std::cout << "Failed to connect to a SpaceMouse, please check if the "
                 "daemon spacenavd is running (as root)"
              << std::endl;
    return 1;
  }
  std::cout << "Successfully connected to a SpaceMouse" << std::endl;

  int n_buttons = std::max(spnav_dev_buttons(), 0);
  std::vector<bool> button_pressed(n_buttons, false);
  std::cout << "Number of buttons: " << n_buttons << std::endl;

  lcm::LCM lcm(config.lcm_url);
  if (!lcm.good()) {
    std::cout << "Failed to initialize LCM" << std::endl;
    spnav_close();
    return 1;
  }
  std::cout << "Initialized LCM" << std::endl;

  spnav_event event{};
  int no_motion_count = 0;
  double mapped_linear[3] = {0, 0, 0};
  double mapped_angular[3] = {0, 0, 0};

  auto next_time = std::chrono::steady_clock::now();
  spacemouse::lcmt_spacemouse_state state;

  while (running) {
    auto ret = spnav_poll_event(&event);
    switch (ret) {
      case 0:
        if (++no_motion_count >
            static_cast<int>(config.settings.static_count_threshold)) {
          if (config.settings.zero_when_static) {
            ZeroStaticAxes(event, config.settings, mapped_linear,
                           mapped_angular);
          }
          no_motion_count = 0;
        }
        break;
      case SPNAV_EVENT_MOTION: {
        const double raw_linear[] = {
            event.motion.x / config.settings.full_scale,
            event.motion.y / config.settings.full_scale,
            event.motion.z / config.settings.full_scale,
        };
        const double raw_angular[] = {
            event.motion.rx / config.settings.full_scale,
            event.motion.ry / config.settings.full_scale,
            event.motion.rz / config.settings.full_scale,
        };
        ApplyAxisMapping(raw_linear, config.settings.linear_axis,
                         config.settings.linear_sign, mapped_linear);
        ApplyAxisMapping(raw_angular, config.settings.angular_axis,
                         config.settings.angular_sign, mapped_angular);
        no_motion_count = 0;
        ClampInvalidAxes(mapped_linear);
        ClampInvalidAxes(mapped_angular);
        break;
      }
      case SPNAV_EVENT_BUTTON:
        if (event.button.bnum >= 0 && event.button.bnum < n_buttons) {
          button_pressed[event.button.bnum] = event.button.press;
        }
        if (event.button.press) {
          handle_button_press(event.button.bnum, lcm);
        }
        break;
      default:
        std::cout << "Received an unknown event from the SpaceMouse, it should "
                     "not happen"
                  << std::endl;
        running = false;
        break;
    }

    auto sys_now = std::chrono::system_clock::now();
    state.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                      sys_now.time_since_epoch())
                      .count();

    for (int i = 0; i < 3; ++i) {
      state.linear[i] = mapped_linear[i];
      state.angular[i] = mapped_angular[i];
    }
    state.n_buttons = n_buttons;
    state.button_pressed.resize(n_buttons);

    for (int i = 0; i < n_buttons; i++) {
      state.button_pressed[i] = button_pressed[i];
    }
    lcm.publish(config.state_lcm_channel, &state);

    next_time +=
        std::chrono::duration_cast<std::chrono::microseconds>(state_period);
    std::this_thread::sleep_until(next_time);
  }

  spnav_close();
  std::cout << "Spacenav closed and program exited gracefully.\n";
  return 0;
}
