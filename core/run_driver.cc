#include "configs/spacemouse_settings.h"
#include "drake/common/yaml/yaml_io.h"
#include "spacemouse/lcmt_spacemouse_state.hpp"
#include <chrono>
#include <csignal>
#include <gflags/gflags.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <spnav.h>
#include <thread>

std::atomic<bool> running(true);

void signalHandler(int signum) {
  std::cout << "\nInterrupt signal (" << signum << ") received.\n";
  running = false; // Stop the loop gracefully
}

DEFINE_string(lcm_channel, "SPACE_MOUSE_TWIST",
              "LCM channel to publish twist messages");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(settings_file_path, "configs/spacemouse_settings.yaml",
              "YAML file containing the settings for the SpaceMouse");
DEFINE_int32(rate, 2000, "Rate to publish state messages (Hz)");
int main(int argc, char **argv) {
  // Parse the settings file
  auto settings =
      drake::yaml::LoadYamlFile<SpacemouseSettings>(FLAGS_settings_file_path);
  const auto period = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::duration<double>(1.0 / FLAGS_rate));

  // Try to connect to a SpaceMouse
  if (spnav_open() == -1) {
    std::cout << "Failed to connect to a SpaceMouse, please check if the "
                 "daemon spacenavd is running (as root)"
              << std::endl;
    return 1;
  }
  std::cout << "Successfully connected to a SpaceMouse" << std::endl;

  int n_buttons = spnav_dev_buttons();
  std::vector<bool> button_pressed(n_buttons, false);
  std::cout << "Number of buttons: " << n_buttons << std::endl;

  // Initialize LCM
  lcm::LCM lcm(FLAGS_lcm_url);
  if (!lcm.good()) {
    std::cout << "Failed to initialize LCM" << std::endl;
    return 1;
  }
  std::cout << "Initialized LCM" << std::endl;

  // Run a loop to publish twist messages
  // - Poll for any events from the SpaceMouse
  // - Publish twist messages if any events are received
  spnav_event sev;
  int no_motion_count = 0;
  double normed_x = 0;
  double normed_y = 0;
  double normed_z = 0;
  double normed_rx = 0;
  double normed_ry = 0;
  double normed_rz = 0;

  auto next_time = std::chrono::steady_clock::now();

  while (running) {
    std::signal(SIGINT, signalHandler);
    auto ret = spnav_poll_event(&sev);
    switch (ret) {
    case 0:
      // After a certain number of polls with no motion, the device is
      // considered static if the flag zero_when_static is set and the motion is
      // less than the static deadband, the linear and angular velocities are
      // set to zero to prevent drift.
      if (++no_motion_count >
          static_cast<int>(settings.static_count_threshold)) {
        if (settings.zero_when_static &&
            std::abs(sev.motion.x) < settings.static_trans_deadband &&
            std::abs(sev.motion.y) < settings.static_trans_deadband &&
            std::abs(sev.motion.z) < settings.static_trans_deadband) {

          normed_x = 0;
          normed_y = 0;
          normed_z = 0;
        }
        if (settings.zero_when_static &&
            std::abs(sev.motion.rx) < settings.static_rot_deadband &&
            std::abs(sev.motion.ry) < settings.static_rot_deadband &&
            std::abs(sev.motion.rz) < settings.static_rot_deadband) {
          normed_rx = 0;
          normed_ry = 0;
          normed_rz = 0;
        }
        no_motion_count = 0; // Reset the no motion count
      }
      break;
    case SPNAV_EVENT_MOTION:
      normed_x = sev.motion.z / settings.full_scale;
      normed_y = -sev.motion.x / settings.full_scale;
      normed_z = sev.motion.y / settings.full_scale;

      normed_rx = sev.motion.rz / settings.full_scale;
      normed_ry = -sev.motion.rx / settings.full_scale;
      normed_rz = sev.motion.ry / settings.full_scale;

      no_motion_count = 0; // Reset the no motion count
      break;
    case SPNAV_EVENT_BUTTON:
      button_pressed[sev.button.bnum] = sev.button.press;
      break;
    default:
      std::cout << "Received an unknown event from the SpaceMouse, it should "
                   "not happen"
                << std::endl;
      running = false;
      break;
    }

    // Fill the state LCM message and publish it
    spacemouse::lcmt_spacemouse_state state;

    auto now = std::chrono::system_clock::now();
    std::time_t utime = std::chrono::system_clock::to_time_t(now);
    state.utime = utime;

    state.linear[0] = normed_x * settings.linear_scale[0];
    state.linear[1] = normed_y * settings.linear_scale[1];
    state.linear[2] = normed_z * settings.linear_scale[2];
    state.angular[0] = normed_rx * settings.angular_scale[0];
    state.angular[1] = normed_ry * settings.angular_scale[1];
    state.angular[2] = normed_rz * settings.angular_scale[2];
    state.n_buttons = n_buttons;
    state.button_pressed.resize(n_buttons);

    for (int i = 0; i < n_buttons; i++) {
      state.button_pressed[i] = button_pressed[i];
    }
    lcm.publish(FLAGS_lcm_channel, &state);

    // Sleep for the remaining time to reach the desired rate
    next_time += period;
    std::this_thread::sleep_until(next_time);
  }

  // Clean up before exiting
  spnav_close();
  std::cout << "Spacenav closed and program exited gracefully.\n";
  return 0;
}
