#include "configs/spacemouse_settings.h"
#include "drake/common/yaml/yaml_io.h"
#include "spacemouse/lcmt_spacemouse_state.hpp"
#include "spacemouse/lcmt_ur_command.hpp"
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

spacemouse::lcmt_ur_command
construct_ur_command(const spacemouse::lcmt_spacemouse_state &state,
                     const SpacemouseSettings &settings) {
  spacemouse::lcmt_ur_command command;
  command.utime = state.utime;
  command.control_mode_expected = spacemouse::lcmt_ur_command::kTCPVelocity;
  for (int i = 0; i < 3; i++) {
    command.tcp_velocity[i] = state.linear[i] * settings.ur_linear_scale[i];
  }
  for (int i = 3; i < 6; i++) {
    command.tcp_velocity[i] =
        state.angular[i - 3] * settings.ur_angular_scale[i - 3];
  }

  // Set the joint positions, velocities, and TCP pose to zero
  // even though they are not used for TCP velocity control
  for (int i = 0; i < 6; i++) {
    command.tcp_pose[i] = 0;
    command.joint_position[i] = 0;
    command.joint_velocity[i] = 0;
  }
  return command;
}

DEFINE_string(state_lcm_channel, "SPACE_MOUSE_0_STATE",
              "LCM channel to publish state messages");
DEFINE_string(robot_command_lcm_channel, "UR_COMMAND",
              "LCM channel to publish robot command messages");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(settings_file_path, "configs/spacemouse_settings.yaml",
              "YAML file containing the settings for the SpaceMouse");
DEFINE_int32(state_publish_rate, 2000, "Rate to publish state messages (Hz)");
DEFINE_string(robot_name, "UR10", "Name of the robot");
DEFINE_int32(robot_command_rate, 130,
             "Rate to publish robot command messages (Hz)");
int main(int argc, char **argv) {
  // Parse the settings file
  auto settings =
      drake::yaml::LoadYamlFile<SpacemouseSettings>(FLAGS_settings_file_path);
  const auto state_period =
      std::chrono::duration<double>(1.0 / FLAGS_state_publish_rate);
  const auto command_period =
      std::chrono::duration<double>(1.0 / FLAGS_robot_command_rate);

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
  auto last_command_pub_time = next_time;
  spacemouse::lcmt_spacemouse_state state;

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
        if (settings.zero_when_static) {
          // Check translational axes (x, y, z)
          int *motion_values[] = {&sev.motion.x, &sev.motion.y, &sev.motion.z};
          double *normed_values[] = {&normed_x, &normed_y, &normed_z};
          for (int i = 0; i < 3; ++i) {
            if (std::abs(*motion_values[i]) < settings.static_trans_deadband) {
              *normed_values[i] = 0;
            }
          }

          // Check rotational axes (rx, ry, rz)
          int *rot_motion_values[] = {&sev.motion.rx, &sev.motion.ry,
                                      &sev.motion.rz};
          double *rot_normed_values[] = {&normed_rx, &normed_ry, &normed_rz};
          for (int i = 0; i < 3; ++i) {
            if (std::abs(*rot_motion_values[i]) <
                settings.static_rot_deadband) {
              *rot_normed_values[i] = 0;
            }
          }
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

    // Get the system time in microseconds
    auto sys_now = std::chrono::system_clock::now();
    state.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                      sys_now.time_since_epoch())
                      .count();

    state.linear[0] = normed_x;
    state.linear[1] = normed_y;
    state.linear[2] = normed_z;
    state.angular[0] = normed_rx;
    state.angular[1] = normed_ry;
    state.angular[2] = normed_rz;
    state.n_buttons = n_buttons;
    state.button_pressed.resize(n_buttons);

    for (int i = 0; i < n_buttons; i++) {
      state.button_pressed[i] = button_pressed[i];
    }
    lcm.publish(FLAGS_state_lcm_channel, &state);

    // Construct the robot command and publish it
    auto now = std::chrono::steady_clock::now();
    if (now - last_command_pub_time >= command_period) {
      if (FLAGS_robot_name == "UR10") {
        spacemouse::lcmt_ur_command command =
            construct_ur_command(state, settings);
        lcm.publish(FLAGS_robot_command_lcm_channel, &command);
      } else {
        std::cout << "Robot name " << FLAGS_robot_name << " not supported"
                  << std::endl;
        running = false;
        break;
      }
      last_command_pub_time = now;
    }

    // Maintain base loop rate
    next_time +=
        std::chrono::duration_cast<std::chrono::microseconds>(state_period);
    std::this_thread::sleep_until(next_time);
  }

  // Clean up before exiting
  spnav_close();
  std::cout << "Spacenav closed and program exited gracefully.\n";
  return 0;
}
