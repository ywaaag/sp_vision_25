#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "example_interfaces/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "io/ros2/ros2.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  tools::Exiter exiter;
  io::ROS2 ros2;
  auto test_node = std::make_shared<rclcpp::Node>("nav_cmd_test_publisher");
  auto chassis_status_publisher =
    test_node->create_publisher<example_interfaces::msg::UInt8>("cmd_chassis_status", 10);
  auto sentry_status_publisher =
    test_node->create_publisher<example_interfaces::msg::UInt8>("cmd_sentry_status", 10);
  auto cmd_vel_publisher = test_node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(test_node);

  int i = 0;
  while (!exiter.exit()) {
    example_interfaces::msg::UInt8 chassis_status_msg;
    chassis_status_msg.data = 1;
    chassis_status_publisher->publish(chassis_status_msg);

    example_interfaces::msg::UInt8 sentry_status_msg;
    sentry_status_msg.data = 2;
    sentry_status_publisher->publish(sentry_status_msg);

    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = 1.0;
    cmd_vel_msg.linear.y = 2.0;
    cmd_vel_msg.angular.z = 3.0;
    cmd_vel_publisher->publish(cmd_vel_msg);

    i++;
    executor->spin_some();
    ros2.spin_some();
    tools::logger()->info(
      "loop={}, chassis_status={}, sentry_status={}, cmd_vel=({}, {}, {})", i,
      ros2.getChassisStatus(), ros2.getSentryStatus(), ros2.getCmdVelX(), ros2.getCmdVelY(),
      ros2.getCmdVelZ());

    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (i > 1000) break;
  }

  return 0;
}
