#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <string>

#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>
#include <optional>

#include "common.h"
#include "configs/spacemouse_settings.h"
#include "core/spacemouse_state_publisher.h"
#include "spacemouse/lcmt_robot_output.hpp"

#include "drake/common/yaml/yaml_io.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

DEFINE_string(state_lcm_channel, "SPACE_MOUSE_FRANKA_STATE",
              "LCM channel to publish state messages");
DEFINE_string(gripper_state_lcm_channel, "PANDA_HAND_STATUS",
              "LCM channel to receive gripper state messages");
DEFINE_string(gripper_command_lcm_channel, "PANDA_HAND_COMMAND",
              "LCM channel to send gripper command messages");
DEFINE_int32(gripper_offset, 20,
             "Offset (in mm) to be added to gripper position");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(settings_file_path, "configs/franka_spacemouse_settings.yaml",
              "YAML file containing the settings for the SpaceMouse");
DEFINE_int32(state_publish_rate, 2000, "Rate to publish state messages (Hz)");
DEFINE_bool(is_simulation, true, "Get robot hand status from simulation");

class FrankaHandStateListener {
 public:
  FrankaHandStateListener(const std::string& lcm_url,
                          const std::string& lcm_gripper_state_channel,
                          bool is_simulation)
      : lcm_(lcm_url),
        lcm_gripper_state_channel_(lcm_gripper_state_channel),
        is_simulation_(is_simulation) {}

  void start() {
    lcm::Subscription* sub;
    if (!is_simulation_) {
      sub = lcm_.subscribe(lcm_gripper_state_channel_,
                           &FrankaHandStateListener::handleGripperStateSchunk,
                           this);
    } else {
      sub = lcm_.subscribe(lcm_gripper_state_channel_,
                           &FrankaHandStateListener::handleGripperState, this);
    }
    sub->setQueueCapacity(100);
  }

  std::optional<int> get_gripper_position() {
    while (lcm_.handleTimeout(0) > 0) {
    }

    if (is_simulation_ && !last_gripper_state_.has_value()) {
      std::cout << "No gripper state received yet." << std::endl;
      return std::nullopt;
    }
    if (!is_simulation_ && !last_gripper_schunk_state_.has_value()) {
      std::cout << "No gripper state received yet." << std::endl;
      return std::nullopt;
    }

    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
    if (!is_simulation_) {
      return last_gripper_schunk_state_->actual_position_mm;
    }
    return static_cast<int>(std::round(
        (last_gripper_state_->position[0] + last_gripper_state_->position[1]) *
        1000));
  }

 private:
  void handleGripperState(const lcm::ReceiveBuffer*, const std::string&,
                          const spacemouse::lcmt_robot_output* msg) {
    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
    last_gripper_state_ = *msg;
  }

  void handleGripperStateSchunk(const lcm::ReceiveBuffer*, const std::string&,
                                const drake::lcmt_schunk_wsg_status* msg) {
    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
    last_gripper_schunk_state_ = *msg;
  }

  lcm::LCM lcm_;
  std::string lcm_gripper_state_channel_;
  std::optional<spacemouse::lcmt_robot_output> last_gripper_state_;
  std::optional<drake::lcmt_schunk_wsg_status> last_gripper_schunk_state_;
  std::mutex gripper_state_mutex_;
  bool is_simulation_;
};

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  auto find_runfile = FindRunfile(FLAGS_settings_file_path);
  if (!find_runfile.has_value()) {
    std::cerr << "Could not find settings file at path: "
              << FLAGS_settings_file_path << std::endl;
    return 1;
  }

  FrankaHandStateListener hand_listener(
      FLAGS_lcm_url, FLAGS_gripper_state_lcm_channel, FLAGS_is_simulation);
  hand_listener.start();

  SpacemouseStatePublisherConfig config;
  config.lcm_url = FLAGS_lcm_url;
  config.state_lcm_channel = FLAGS_state_lcm_channel;
  config.state_publish_rate = FLAGS_state_publish_rate;
  config.settings =
      drake::yaml::LoadYamlFile<SpacemouseSettings>(find_runfile.value());

  return RunSpacemouseStatePublisher(
      config, [&hand_listener](int button, lcm::LCM& lcm) {
        if (button != 0 && button != 1) {
          return;
        }

        auto current_position = hand_listener.get_gripper_position();
        if (!current_position.has_value()) {
          return;
        }

        drake::lcmt_schunk_wsg_command command;
        command.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();

        if (button == 0) {
          command.target_position_mm =
              std::min(current_position.value() + FLAGS_gripper_offset, 80);
        } else {
          command.target_position_mm =
              std::max(current_position.value() - FLAGS_gripper_offset, 0);
        }

        lcm.publish(FLAGS_gripper_command_lcm_channel, &command);
      });
}
