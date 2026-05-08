#include "configs/spacemouse_settings.h"
#include "spacemouse/lcmt_spacemouse_state.hpp"
#include "spacemouse/lcmt_ur_command.hpp"
#include "robotiq/lcmt_robotiq_command.hpp"
#include "robotiq/lcmt_robotiq_status.hpp"

#include "drake/common/yaml/yaml_io.h"

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <mutex>
#include <optional>

#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>
#include <spnav.h>

std::atomic<bool> running(true);

void signalHandler(int signum) {
  std::cout << "\nInterrupt signal (" << signum << ") received.\n";
  running = false;
}

spacemouse::lcmt_ur_command
construct_ur_command(const spacemouse::lcmt_spacemouse_state &state,
                     const URSpacemouseSettings &settings) {
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

  for (int i = 0; i < 6; i++) {
    command.tcp_pose[i] = 0;
    command.joint_position[i] = 0;
    command.joint_velocity[i] = 0;
  }
  return command;
}

DEFINE_string(state_lcm_channel, "SPACE_MOUSE_UR_STATE",
              "LCM channel to publish state messages");
DEFINE_string(robot_command_lcm_channel, "UR_ROBOT_COMMAND",
              "LCM channel to publish robot command messages");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(settings_file_path, "configs/spacemouse_settings.yaml",
              "YAML file containing the settings for the SpaceMouse");
DEFINE_int32(state_publish_rate, 2000, "Rate to publish state messages (Hz)");
DEFINE_string(robot_name, "UR10", "Name of the robot");
DEFINE_int32(robot_command_rate, 130,
             "Rate to publish robot command messages (Hz)");
DEFINE_string(gripper_command_lcm_channel, "ROBOTIQ_COMMAND",
              "LCM channel to send gripper command messages");
DEFINE_string(gripper_status_lcm_channel, "ROBOTIQ_STATUS",
              "LCM channel to receive gripper status messages");
DEFINE_int32(gripper_offset, 20, "Offset to be added to gripper position (position range is 0 - 255)");
DEFINE_int32(gripper_speed, 100, "Speed of the gripper");
DEFINE_int32(gripper_force, 0, "Force of the gripper");

class RobotiqGripperStateListener {
 public:
  RobotiqGripperStateListener(const std::string& lcm_url,
                              const std::string& lcm_status_channel)
      : lcm_(lcm_url),
        lcm_status_channel_(lcm_status_channel) {}
  ~RobotiqGripperStateListener() {}

  int start() {
    lcm::Subscription* sub =
        lcm_.subscribe(lcm_status_channel_,
                       &RobotiqGripperStateListener::handleGripperStatus, this);
    sub->setQueueCapacity(100);
    return 0;
  }

  std::optional<robotiq::lcmt_robotiq_status> get_gripper_status() {
    while (lcm_.handleTimeout(0) > 0) {
    }

    std::lock_guard<std::mutex> lock(gripper_status_mutex_);
    return last_gripper_status_;
  }

 private:
  void handleGripperStatus(const lcm::ReceiveBuffer* rbuf,
                          const std::string& channel,
                          const robotiq::lcmt_robotiq_status* msg) {
    std::lock_guard<std::mutex> lock(gripper_status_mutex_);
    last_gripper_status_ = *msg;
  }

  lcm::LCM lcm_;
  std::string lcm_status_channel_;
  std::optional<robotiq::lcmt_robotiq_status> last_gripper_status_;
  std::mutex gripper_status_mutex_;
};

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  auto settings =
      drake::yaml::LoadYamlFile<URSpacemouseSettings>(FLAGS_settings_file_path);
  const auto state_period =
      std::chrono::duration<double>(1.0 / FLAGS_state_publish_rate);
  const auto command_period =
      std::chrono::duration<double>(1.0 / FLAGS_robot_command_rate);

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

  lcm::LCM lcm(FLAGS_lcm_url);
  if (!lcm.good()) {
    std::cout << "Failed to initialize LCM" << std::endl;
    return 1;
  }
  std::cout << "Initialized LCM" << std::endl;

  // Initialize gripper state listener
  RobotiqGripperStateListener gripper_listener(FLAGS_lcm_url,
                                                FLAGS_gripper_status_lcm_channel);
  gripper_listener.start();

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
  robotiq::lcmt_robotiq_command gripper_command;

  double *normed_values[] = {&normed_x, &normed_y, &normed_z};
  double *rot_normed_values[] = {&normed_rx, &normed_ry, &normed_rz};

  while (running) {
    std::signal(SIGINT, signalHandler);
    auto ret = spnav_poll_event(&sev);
    switch (ret) {
    case 0:
      if (++no_motion_count >
          static_cast<int>(settings.static_count_threshold)) {
        if (settings.zero_when_static) {
          int *motion_values[] = {&sev.motion.x, &sev.motion.y, &sev.motion.z};
          for (int i = 0; i < 3; ++i) {
            if (std::abs(*motion_values[i]) < settings.static_trans_deadband) {
              *normed_values[i] = 0;
            }
          }

          int *rot_motion_values[] = {&sev.motion.rx, &sev.motion.ry,
                                      &sev.motion.rz};
          for (int i = 0; i < 3; ++i) {
            if (std::abs(*rot_motion_values[i]) <
                settings.static_rot_deadband) {
              *rot_normed_values[i] = 0;
            }
          }
        }
        no_motion_count = 0;
      }
      break;
    case SPNAV_EVENT_MOTION:
      normed_x = sev.motion.z / settings.full_scale;
      normed_y = -sev.motion.x / settings.full_scale;
      normed_z = sev.motion.y / settings.full_scale;

      normed_rx = sev.motion.rz / settings.full_scale;
      normed_ry = -sev.motion.rx / settings.full_scale;
      normed_rz = sev.motion.ry / settings.full_scale;

      no_motion_count = 0;

      for (int i = 0; i < 3; i++) {
        if (*normed_values[i] < -1.0 || *normed_values[i] > 1.0) {
          *normed_values[i] = 0.0;
        }
        if (*rot_normed_values[i] < -1.0 || *rot_normed_values[i] > 1.0) {
          *rot_normed_values[i] = 0.0;
        }
      }
      break;
    case SPNAV_EVENT_BUTTON:
      button_pressed[sev.button.bnum] = sev.button.press;
      if (!sev.button.press) {
        break;
      } else {
      auto gripper_status_opt = gripper_listener.get_gripper_status();
      if (!gripper_status_opt.has_value()) {
        break;
      }

      auto status = gripper_status_opt.value();
      // if ((status.goto_status != 0) && (status.object_detection_status == 0)) {
      //   // Skip processing button events while the gripper is moving
      //   break;
      // }

      gripper_command.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count();
      gripper_command.speed = FLAGS_gripper_speed;
      gripper_command.force = FLAGS_gripper_force;

      if (sev.button.bnum == 0) {
        // Open gripper
        gripper_command.position = std::min(
            static_cast<int>(status.position) + FLAGS_gripper_offset, 255);
      } else if (sev.button.bnum == 1) {
        // Close gripper
        gripper_command.position = std::max(
            static_cast<int>(status.position) - FLAGS_gripper_offset, 0);
      }

      lcm.publish(FLAGS_gripper_command_lcm_channel, &gripper_command);
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

    state.linear[0] = -normed_y;
    state.linear[1] = normed_x;
    state.linear[2] = normed_z;
    state.angular[0] = -normed_ry;
    state.angular[1] = normed_rx;
    state.angular[2] = normed_rz;
    state.n_buttons = n_buttons;
    state.button_pressed.resize(n_buttons);

    for (int i = 0; i < n_buttons; i++) {
      state.button_pressed[i] = button_pressed[i];
    }
    lcm.publish(FLAGS_state_lcm_channel, &state);

    auto now = std::chrono::steady_clock::now();
    if (now - last_command_pub_time >= command_period) {
      if (FLAGS_robot_name == "UR10") {
        spacemouse::lcmt_ur_command command =
            construct_ur_command(state, settings);
        // lcm.publish(FLAGS_robot_command_lcm_channel, &command);
      } else {
        std::cout << "Robot name " << FLAGS_robot_name << " not supported"
                  << std::endl;
        running = false;
        break;
      }
      last_command_pub_time = now;
    }

    next_time +=
        std::chrono::duration_cast<std::chrono::microseconds>(state_period);
    std::this_thread::sleep_until(next_time);
  }

  spnav_close();
  std::cout << "Spacenav closed and program exited gracefully.\n";
  return 0;
}
