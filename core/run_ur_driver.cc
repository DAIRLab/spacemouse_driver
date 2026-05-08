#include <algorithm>
#include <chrono>
#include <mutex>
#include <string>

#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>
#include <optional>

#include "configs/spacemouse_settings.h"
#include "core/spacemouse_state_publisher.h"
#include "robotiq/lcmt_robotiq_command.hpp"
#include "robotiq/lcmt_robotiq_status.hpp"

#include "drake/common/yaml/yaml_io.h"

DEFINE_string(state_lcm_channel, "SPACE_MOUSE_UR_STATE",
              "LCM channel to publish state messages");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(settings_file_path, "configs/ur_spacemouse_settings.yaml",
              "YAML file containing the settings for the SpaceMouse");
DEFINE_int32(state_publish_rate, 2000, "Rate to publish state messages (Hz)");
DEFINE_string(gripper_command_lcm_channel, "ROBOTIQ_COMMAND",
              "LCM channel to send gripper command messages");
DEFINE_string(gripper_status_lcm_channel, "ROBOTIQ_STATUS",
              "LCM channel to receive gripper status messages");
DEFINE_int32(
    gripper_offset, 20,
    "Offset to be added to gripper position (position range is 0 - 255)");
DEFINE_int32(gripper_speed, 100, "Speed of the gripper");
DEFINE_int32(gripper_force, 0, "Force of the gripper");

class RobotiqGripperStateListener {
 public:
  RobotiqGripperStateListener(const std::string& lcm_url,
                              const std::string& lcm_status_channel)
      : lcm_(lcm_url), lcm_status_channel_(lcm_status_channel) {}

  void start() {
    lcm::Subscription* sub =
        lcm_.subscribe(lcm_status_channel_,
                       &RobotiqGripperStateListener::handleGripperStatus, this);
    sub->setQueueCapacity(100);
  }

  std::optional<robotiq::lcmt_robotiq_status> get_gripper_status() {
    while (lcm_.handleTimeout(0) > 0) {
    }

    std::lock_guard<std::mutex> lock(gripper_status_mutex_);
    return last_gripper_status_;
  }

 private:
  void handleGripperStatus(const lcm::ReceiveBuffer*, const std::string&,
                           const robotiq::lcmt_robotiq_status* msg) {
    std::lock_guard<std::mutex> lock(gripper_status_mutex_);
    last_gripper_status_ = *msg;
  }

  lcm::LCM lcm_;
  std::string lcm_status_channel_;
  std::optional<robotiq::lcmt_robotiq_status> last_gripper_status_;
  std::mutex gripper_status_mutex_;
};

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  RobotiqGripperStateListener gripper_listener(
      FLAGS_lcm_url, FLAGS_gripper_status_lcm_channel);
  gripper_listener.start();

  SpacemouseStatePublisherConfig config;
  config.lcm_url = FLAGS_lcm_url;
  config.state_lcm_channel = FLAGS_state_lcm_channel;
  config.state_publish_rate = FLAGS_state_publish_rate;
  config.settings =
      drake::yaml::LoadYamlFile<SpacemouseSettings>(FLAGS_settings_file_path);

  return RunSpacemouseStatePublisher(config, [&gripper_listener](
                                                 int button, lcm::LCM& lcm) {
    auto gripper_status = gripper_listener.get_gripper_status();
    if (!gripper_status.has_value()) {
      return;
    }

    robotiq::lcmt_robotiq_command command;
    command.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
    command.speed = FLAGS_gripper_speed;
    command.force = FLAGS_gripper_force;

    if (button == 0) {
      command.position = std::min(
          static_cast<int>(gripper_status->position) + FLAGS_gripper_offset,
          255);
    } else if (button == 1) {
      command.position = std::max(
          static_cast<int>(gripper_status->position) - FLAGS_gripper_offset, 0);
    } else {
      return;
    }

    lcm.publish(FLAGS_gripper_command_lcm_channel, &command);
  });
}
