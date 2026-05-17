#include "subscribenavcmd.hpp"

namespace io {

SubscribeNavCmd::SubscribeNavCmd() : Node("nav_cmd_subscriber")
{
    auto qos = rclcpp::QoS(1);
    cmd_chassis_status_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
        "cmd_chassis_status", qos,
        std::bind(&SubscribeNavCmd::cmdChassisStatusCallback, this, std::placeholders::_1));

    cmd_sentry_status_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
        "cmd_sentry_status", qos,
        std::bind(&SubscribeNavCmd::cmdSentryStatusCallback, this, std::placeholders::_1));

    terrain_status_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "cmd_terrain_state", qos,
        std::bind(&SubscribeNavCmd::terrainStatusCallback, this, std::placeholders::_1));

    target_mode_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
        "cmd_target_mode", qos,
        std::bind(&SubscribeNavCmd::targetModeCallback, this, std::placeholders::_1));

    bump_status_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
        "cmd_bump_status", qos,
        std::bind(&SubscribeNavCmd::bumpStatusCallback, this, std::placeholders::_1));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", qos,
        std::bind(&SubscribeNavCmd::cmdVelCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "nav_cmd_subscriber initialized.");
}

void SubscribeNavCmd::cmdChassisStatusCallback(const example_interfaces::msg::UInt8::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_chassis_status_ = msg->data;
}

void SubscribeNavCmd::cmdSentryStatusCallback(const example_interfaces::msg::UInt8::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_sentry_status_ = msg->data;
}

void SubscribeNavCmd::targetModeCallback(const example_interfaces::msg::UInt8::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_target_mode_ = msg->data;
}

void SubscribeNavCmd::terrainStatusCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_terrain_status_ = msg->data;
}

void SubscribeNavCmd::bumpStatusCallback(const example_interfaces::msg::UInt8::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_bump_status_ = msg->data;
}

void SubscribeNavCmd::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cmd_vel_ = *msg;
}

uint8_t SubscribeNavCmd::getChassisStatus()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_chassis_status_;
}

uint8_t SubscribeNavCmd::getSentryStatus()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_sentry_status_;
}

uint8_t SubscribeNavCmd::getTargetMode()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_target_mode_;
}

uint8_t SubscribeNavCmd::getTerrainStatus()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_terrain_status_;
}

uint8_t SubscribeNavCmd::getBumpStatus()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_bump_status_;
}

float SubscribeNavCmd::getCmdVelX()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_cmd_vel_.linear.x;
}

float SubscribeNavCmd::getCmdVelY()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_cmd_vel_.linear.y;
}

float SubscribeNavCmd::getCmdVelZ()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_cmd_vel_.angular.z;
}


} // namespace io
