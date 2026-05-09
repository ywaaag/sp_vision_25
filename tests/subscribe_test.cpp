#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "io/ros2/ros2.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  tools::Exiter exiter;
  io::ROS2 ros2;

  while (!exiter.exit()) {
    ros2.spin_some();
    tools::logger()->info(
      "chassis_status={}, sentry_status={}, cmd_vel=({}, {}, {})", ros2.getChassisStatus(),
      ros2.getSentryStatus(), ros2.getCmdVelX(), ros2.getCmdVelY(), ros2.getCmdVelZ());
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }
  return 0;
}
