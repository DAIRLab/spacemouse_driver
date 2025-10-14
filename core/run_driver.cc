#include "spacemouse/lcmt_twist.hpp"
#include <chrono>
#include <gflags/gflags.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <spnav.h>

DEFINE_string(lcm_channel, "SPACE_MOUSE_TWIST",
              "LCM channel to publish twist messages");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=1",
              "LCM URL with IP, port, and TTL settings");
int main(int argc, char **argv) {
  // Try to connect to a SpaceMouse
  if (spnav_open() == -1) {
    std::cout << "Failed to connect to a SpaceMouse, please check if the "
                 "daemon spacenavd is running"
              << std::endl;
    return 1;
  }
  std::cout << "Successfully connected to a SpaceMouse" << std::endl;

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
  while (true) {
    auto ret = spnav_poll_event(&sev);
    spacemouse::lcmt_twist twist;

    auto now = std::chrono::system_clock::now();
    std::time_t utime = std::chrono::system_clock::to_time_t(now);
    twist.utime = utime;

    twist.linear[0] = sev.motion.x;
    twist.linear[1] = sev.motion.y;
    twist.linear[2] = sev.motion.z;
    twist.angular[0] = sev.motion.rx;
    twist.angular[1] = sev.motion.ry;
    twist.angular[2] = sev.motion.rz;
    lcm.publish(FLAGS_lcm_channel, &twist);
  }
  return 0;
}
