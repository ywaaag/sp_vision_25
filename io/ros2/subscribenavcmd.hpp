#pragma once
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <mutex>

namespace io {

class SubscribeNavCmd : public rclcpp::Node
{
public:
    SubscribeNavCmd();
    
    uint8_t getChassisStatus();
    uint8_t getSentryStatus();
    float getCmdVelX();
    float getCmdVelY();
    float getCmdVelZ();

private:
    void cmdChassisStatusCallback(const example_interfaces::msg::UInt8::SharedPtr msg);
    void cmdSentryStatusCallback(const example_interfaces::msg::UInt8::SharedPtr msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr cmd_chassis_status_sub_;
    rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr cmd_sentry_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    uint8_t latest_chassis_status_ = 0;
    uint8_t latest_sentry_status_ = 0;
    geometry_msgs::msg::Twist latest_cmd_vel_;
    std::mutex mutex_;
};

}  // namespace io