//clang-format off
#include "common.h"
#include "configs/spacemouse_settings.h"
#include "spacemouse/lcmt_franka_cartesian_pose.hpp"
#include "spacemouse/lcmt_robot_output.hpp"
#include "spacemouse/lcmt_spacemouse_state.hpp"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/wrap_to.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
// clang-format on

#include <chrono>
#include <cmath>  // Required for std::acos()
#include <csignal>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Core>
#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>
#include <optional>
#include <spnav.h>
#include <variant>

using drake::math::RollPitchYaw;
using drake::math::RotationMatrix;

std::atomic<bool> running(true);

void signalHandler(int signum) {
  std::cout << "\nInterrupt signal (" << signum << ") received.\n";
  running = false;  // Stop the loop gracefully
}

spacemouse::lcmt_franka_cartesian_pose construct_franka_cartesian_pose(
    int64_t utime, const Eigen::VectorXd& v,
    const SpacemouseSettings& settings) {
  spacemouse::lcmt_franka_cartesian_pose pose;
  pose.utime = utime;
  for (int i = 0; i < 6; i++) {
    pose.target_cartesian_pose[i] = v[i];
  }
  return pose;
}

DEFINE_string(state_lcm_channel, "SPACE_MOUSE_0_STATE",
              "LCM channel to publish state messages");
DEFINE_string(robot_command_lcm_channel, "TARGET_CARTESIAN_POSE",
              "LCM channel to publish robot command messages");
DEFINE_string(robot_status_lcm_channel, "FRANKA_STATE",
              "LCM channel to receive robot status messages");
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
DEFINE_int32(robot_command_rate, 130,
             "Rate to publish robot command messages (Hz)");
DEFINE_bool(is_simulation, true, "Get robot hand status from simulation");

class FrankaCartesianPoseIntegrator {
 public:
  FrankaCartesianPoseIntegrator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context, std::string end_effector_name,
      const std::string& lcm_url, const std::string& lcm_command_channel,
      const std::string& lcm_status_channel, double linear_scale,
      double angular_scale)
      : plant_(plant),
        plant_context_(context),
        end_effector_name_(end_effector_name),
        lcm_(lcm_url),
        lcm_command_channel_(lcm_command_channel),
        lcm_status_channel_(lcm_status_channel),
        linear_scale_(linear_scale),
        angular_scale_(angular_scale) {}
  ~FrankaCartesianPoseIntegrator() {}

  int start() {
    lcm::Subscription* sub =
        lcm_.subscribe(lcm_status_channel_,
                       &FrankaCartesianPoseIntegrator::handleRobotOutput, this);
    sub->setQueueCapacity(100);
    if (lcm_.handleTimeout(2) < 0) {
      drake::log()->error("LCM handle failed to recieve initial robot output.");
      return -1;
    }
    auto current_pose = GetCurrentCartesianPose();
    drake::log()->info("Current pose initialized.");
    DRAKE_DEMAND(current_pose.has_value());
    DRAKE_DEMAND(std::get<0>(current_pose.value()).rotation().IsValid());
    last_predicted_robot_pose_ = std::get<0>(current_pose.value());
    last_predicted_robot_pose_time_ = std::get<1>(current_pose.value());
    last_velocity_command_ = Eigen::VectorXd::Zero(6);
    drake::log()->info("FrankaCartesianPoseIntegrator started.");
    return 0;
  }

  std::optional<std::tuple<drake::math::RigidTransform<double>, double>>
  GetCurrentCartesianPose() {
    // Clear out any other pending messages
    while (lcm_.handleTimeout(0) > 0) {
    }

    if (!last_robot_output_.has_value()) {
      return std::nullopt;
    }
    Eigen::VectorXd joint_positions(7);
    Eigen::VectorXd joint_velocities(7);
    double last_output_time = 0.0;
    {
      std::lock_guard<std::mutex> lock(robot_input_mutex_);
      joint_positions << last_robot_output_->position[0],
          last_robot_output_->position[1], last_robot_output_->position[2],
          last_robot_output_->position[3], last_robot_output_->position[4],
          last_robot_output_->position[5], last_robot_output_->position[6];
      joint_velocities << last_robot_output_->velocity[0],
          last_robot_output_->velocity[1], last_robot_output_->velocity[2],
          last_robot_output_->velocity[3], last_robot_output_->velocity[4],
          last_robot_output_->velocity[5], last_robot_output_->velocity[6];
      last_output_time = last_robot_output_->utime * 1e-6;
    }
    plant_.SetPositions(plant_context_, joint_positions);
    plant_.SetVelocities(plant_context_, joint_velocities);
    return std::make_tuple(
        plant_.CalcRelativeTransform(*plant_context_, plant_.world_frame(),
                                     plant_.GetFrameByName(end_effector_name_)),
        last_output_time);
  }

  std::optional<Eigen::VectorXd> GetTargetCartesianPose(Eigen::VectorXd v) {
    DRAKE_ASSERT(v.size() == 6);
    auto current_pose = GetCurrentCartesianPose();
    if (!current_pose.has_value()) {
      return std::nullopt;
    }
    auto current_transform = std::get<0>(current_pose.value());
    double current_time = std::get<1>(current_pose.value());

    DRAKE_ASSERT(current_time - last_predicted_robot_pose_time_ >= 0);
    DRAKE_DEMAND(current_time - last_predicted_robot_pose_time_ <
                 1e2);  // 0.1 second
    auto predicted_pose = IntegrateVelocityCommand(
        last_predicted_robot_pose_, last_velocity_command_,
        current_time - last_predicted_robot_pose_time_);

    if ((predicted_pose.translation() - current_transform.translation())
            .norm() > max_translation_delta_) {
      drake::log()->debug(
          "Limiting translation delta from {} to {} meters.",
          (predicted_pose.translation() - current_transform.translation())
              .norm(),
          max_translation_delta_);
      auto dx = predicted_pose.translation() - current_transform.translation();
      predicted_pose.set_translation(current_transform.translation() +
                                     dx.normalized() * max_translation_delta_);
    }

    auto rotation_diff = current_transform.rotation().InvertAndCompose(
        predicted_pose.rotation());
    auto axis_angle_diff = rotation_diff.ToAngleAxis();
    if (std::abs(axis_angle_diff.angle()) > max_rotation_delta_) {
      drake::log()->debug("Limiting rotation delta from {} to {} radians.",
                          axis_angle_diff.angle(), max_rotation_delta_);
      auto target_angle_diff =
          (axis_angle_diff.angle() > 0 ? 1 : -1) * max_rotation_delta_;
      auto limited_rotation = RotationMatrix<double>(
          Eigen::AngleAxis<double>(target_angle_diff, axis_angle_diff.axis()));
      predicted_pose.set_rotation(current_transform.rotation() *
                                  limited_rotation);
    }

    auto target_pose =
        IntegrateVelocityCommand(predicted_pose, v);  // plan pose in 1s

    last_predicted_robot_pose_ = predicted_pose;
    last_predicted_robot_pose_time_ = current_time;
    last_velocity_command_ = v;

    Eigen::Vector<double, 6> target_pose_vector;
    target_pose_vector << target_pose.translation(),
        target_pose.rotation().ToRollPitchYaw().vector();
    return target_pose_vector;
  }

  void handleRobotOutput(const lcm::ReceiveBuffer*, const std::string&,
                         const spacemouse::lcmt_robot_output* msg) {
    std::lock_guard<std::mutex> lock(robot_input_mutex_);
    last_robot_output_ = *msg;
  }

 private:
  drake::math::RigidTransform<double> IntegrateVelocityCommand(
      const drake::math::RigidTransform<double>& current_transform,
      const Eigen::VectorXd& v, double dt = 1.0) {
    DRAKE_ASSERT(v.size() == 6);
    Eigen::Vector3d delta_position =
        Eigen::Vector3d(v[0], v[1], v[2]) * linear_scale_ * dt;
    Eigen::Vector3d delta_rpy =
        Eigen::Vector3d(v[3], v[4], v[5]) * angular_scale_ * dt;

    // Update translation
    Eigen::Vector3d new_translation =
        current_transform.translation() + delta_position;

    // Update rotation
    auto new_rotation = RotationMatrix<double>(RollPitchYaw<double>(
                            delta_rpy[0], delta_rpy[1], delta_rpy[2])) *
                        current_transform.rotation();

    return drake::math::RigidTransform<double>(new_rotation, new_translation);
  }

  drake::math::RigidTransform<double> last_predicted_robot_pose_;
  double last_predicted_robot_pose_time_{0.0};
  Eigen::VectorXd last_velocity_command_ = Eigen::VectorXd::Zero(6);
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* plant_context_;
  std::string end_effector_name_;
  std::optional<spacemouse::lcmt_robot_output> last_robot_output_;
  std::mutex robot_input_mutex_;
  lcm::LCM lcm_;
  const std::string lcm_command_channel_;
  const std::string lcm_status_channel_;
  double linear_scale_;
  double angular_scale_;
  double max_translation_delta_{
      0.1};  // Maximum allowed translation delta in meters
  double max_rotation_delta_{M_PI /
                             12};  // Maximum allowed rotation delta in radians
};

class FrankaHandStateListener {
 public:
  FrankaHandStateListener(const std::string& lcm_url,
                          const std::string& lcm_gripper_state_channel,
                          bool is_simulation)
      : lcm_(lcm_url),
        lcm_gripper_state_channel_(lcm_gripper_state_channel),
        is_simulation_(is_simulation) {}
  ~FrankaHandStateListener() {}

  int start() {
    lcm::Subscription* sub;
    if (!is_simulation_)
      sub = lcm_.subscribe(lcm_gripper_state_channel_,
                           &FrankaHandStateListener::handleGripperStateSchunk,
                           this);
    else
      sub = lcm_.subscribe(lcm_gripper_state_channel_,
                           &FrankaHandStateListener::handleGripperState, this);
    sub->setQueueCapacity(100);
    return 0;
  }

  std::optional<int> get_gripper_position() {
    while (lcm_.handleTimeout(0) > 0) {
    }

    if (is_simulation_ && !last_gripper_state_.has_value()) {
      std::cout << "No gripper state received yet." << std::endl;
      return std::nullopt;
    } else if (!is_simulation_ && !last_gripper_schunk_state_.has_value()) {
      std::cout << "No gripper state received yet." << std::endl;
      return std::nullopt;
    }

    int position_in_mm = 0;
    {
      std::lock_guard<std::mutex> lock(gripper_state_mutex_);

      if (!is_simulation_) {
        position_in_mm = last_gripper_schunk_state_->actual_position_mm;
      } else {
        position_in_mm = std::round((last_gripper_state_->position[0] +
                                     last_gripper_state_->position[1]) *
                                    1000);
      }
    }
    return position_in_mm;
  }

 private:
  void handleGripperState(const lcm::ReceiveBuffer* rbuf,
                          const std::string& channel,
                          const spacemouse::lcmt_robot_output* msg) {
    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
    last_gripper_state_ = *msg;
  }

  void handleGripperStateSchunk(const lcm::ReceiveBuffer* rbuf,
                                const std::string& channel,
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

inline const Eigen::Vector3d TOOL_ATTACHMENT_FRAME = {0, 0, 0.107};
inline const drake::math::RigidTransform<double> T_EE_L7 =
    drake::math::RigidTransform<double>(
        drake::math::RotationMatrix<double>(
            drake::math::RollPitchYaw<double>(M_PI, 0, -M_PI / 4)),
        TOOL_ATTACHMENT_FRAME);

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::set_log_level("debug");

  // Parse the settings file
  auto find_runfile = FindRunfile(FLAGS_settings_file_path);
  if (!find_runfile.has_value()) {
    std::cerr << "Could not find settings file at path: "
              << FLAGS_settings_file_path << std::endl;
    return 1;
  }
  auto settings =
      drake::yaml::LoadYamlFile<FrankaSpacemouseSettings>(find_runfile.value());
  const auto state_period =
      std::chrono::duration<double>(1.0 / FLAGS_state_publish_rate);
  const auto command_period =
      std::chrono::duration<double>(1.0 / FLAGS_robot_command_rate);

  std::cout << "Setting up Fraka Cartesian Pose Integrator\n";
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser parser(&plant, nullptr);
  parser.SetAutoRenaming(true);
  drake::multibody::ModelInstanceIndex franka_index = parser.AddModelsFromUrl(
      "package://drake_models/franka_description/urdf/"
      "panda_arm_hand_with_fixed_long_fingers.urdf")[0];
  drake::math::RigidTransform<double> X_WI =
      drake::math::RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   X_WI);
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();
  FrankaCartesianPoseIntegrator integrator(
      plant, plant_context.get(), "finger_tip", FLAGS_lcm_url,
      FLAGS_robot_command_lcm_channel, FLAGS_robot_status_lcm_channel,
      settings.franka_linear_scale, settings.franka_angular_scale);
  std::cout << "Franka Cartesian Pose Integrator setup complete\n";
  integrator.start();

  // Try to connect to the Franka Hand state listener
  FrankaHandStateListener hand_listener(
      FLAGS_lcm_url, FLAGS_gripper_state_lcm_channel, FLAGS_is_simulation);
  hand_listener.start();

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
  drake::lcmt_schunk_wsg_command gripper_command;
  std::optional<int> current_gripper_position;

  while (running) {
    std::signal(SIGINT, signalHandler);
    auto ret = spnav_poll_event(&sev);
    switch (ret) {
      case 0:
        // After a certain number of polls with no motion, the device is
        // considered static if the flag zero_when_static is set and the motion
        // is less than the static deadband, the linear and angular velocities
        // are set to zero to prevent drift.
        if (++no_motion_count >
            static_cast<int>(settings.static_count_threshold)) {
          if (settings.zero_when_static) {
            // Check translational axes (x, y, z)
            int* motion_values[] = {&sev.motion.x, &sev.motion.y,
                                    &sev.motion.z};
            double* normed_values[] = {&normed_x, &normed_y, &normed_z};
            for (int i = 0; i < 3; ++i) {
              if (std::abs(*motion_values[i]) <
                  settings.static_trans_deadband) {
                *normed_values[i] = 0;
              }
            }

            // Check rotational axes (rx, ry, rz)
            int* rot_motion_values[] = {&sev.motion.rx, &sev.motion.ry,
                                        &sev.motion.rz};
            double* rot_normed_values[] = {&normed_rx, &normed_ry, &normed_rz};
            for (int i = 0; i < 3; ++i) {
              if (std::abs(*rot_motion_values[i]) <
                  settings.static_rot_deadband) {
                *rot_normed_values[i] = 0;
              }
            }
          }
          no_motion_count = 0;  // Reset the no motion count
        }
        break;
      case SPNAV_EVENT_MOTION:
        normed_x = sev.motion.z / settings.full_scale;
        normed_y = -sev.motion.x / settings.full_scale;
        normed_z = sev.motion.y / settings.full_scale;

        normed_rx = sev.motion.rz / settings.full_scale;
        normed_ry = -sev.motion.rx / settings.full_scale;
        normed_rz = sev.motion.ry / settings.full_scale;

        no_motion_count = 0;  // Reset the no motion count
        break;
      case SPNAV_EVENT_BUTTON:
        button_pressed[sev.button.bnum] = sev.button.press;
        if (!sev.button.press) {
          // Ignore button release events
          break;
        }
        // Example: Open gripper on button 0 press, close on button 1 press
        gripper_command.utime =
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
        current_gripper_position = hand_listener.get_gripper_position();
        if (!current_gripper_position.has_value()) {
          break;
        }
        if (sev.button.bnum == 0 && sev.button.press) {
          gripper_command.target_position_mm = std::min(
              current_gripper_position.value() + FLAGS_gripper_offset, 80);
        } else if (sev.button.bnum == 1 && sev.button.press) {
          gripper_command.target_position_mm = std::max(
              current_gripper_position.value() - FLAGS_gripper_offset, 0);
        }
        lcm.publish(FLAGS_gripper_command_lcm_channel, &gripper_command);

        break;
      default:
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

    // Construct the Franka cartesian pose command and publish it
    auto now = std::chrono::steady_clock::now();
    if (now - last_command_pub_time >= command_period) {
      Eigen::VectorXd v(6);
      v << state.linear[0], state.linear[1], state.linear[2], state.angular[0],
          state.angular[1], state.angular[2];
      auto pose_opt = integrator.GetTargetCartesianPose(v);
      if (pose_opt.has_value()) {
        spacemouse::lcmt_franka_cartesian_pose pose =
            construct_franka_cartesian_pose(state.utime, pose_opt.value(),
                                            settings);
        lcm.publish(FLAGS_robot_command_lcm_channel, &pose);
        last_command_pub_time = now;
      } else {
        drake::log()->warn(
            "No target pose computed, skipping command publish.");
      }
    }

    // Maintain base loop rate
    next_time +=
        std::chrono::duration_cast<std::chrono::microseconds>(state_period);
    std::this_thread::sleep_until(next_time);
  }

  std::cout << "Preparing to exit, sending final command to stop the robot\n";
  // Send a final command to stop the robot
  auto pose_opt = integrator.GetTargetCartesianPose(Eigen::VectorXd::Zero(6));
  if (pose_opt.has_value()) {
    spacemouse::lcmt_franka_cartesian_pose pose =
        construct_franka_cartesian_pose(state.utime, pose_opt.value(),
                                        settings);
    lcm.publish(FLAGS_robot_command_lcm_channel, &pose);
  }

  // Clean up before exiting
  spnav_close();
  std::cout << "Spacenav closed and program exited gracefully.\n";
  return 0;
}