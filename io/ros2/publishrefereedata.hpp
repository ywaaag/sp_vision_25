#pragma once

#include "rclcpp/rclcpp.hpp"
#include "combat_rm_interfaces/msg/game_status.hpp"
#include "combat_rm_interfaces/msg/game_robot_hp.hpp"
#include "combat_rm_interfaces/msg/event_data.hpp"
#include "combat_rm_interfaces/msg/robot_status.hpp"
#include "combat_rm_interfaces/msg/robot_pos.hpp"
#include "combat_rm_interfaces/msg/hurt_data.hpp"
#include "combat_rm_interfaces/msg/rfid_status.hpp"
#include "combat_rm_interfaces/msg/ground_robot_position.hpp"
#include "combat_rm_interfaces/msg/sentry_gimbal.hpp"
#include "combat_rm_interfaces/msg/sentry_info.hpp"

#include "io/packet_typedef.hpp"

namespace io
{

class PublishRefereeData : public rclcpp::Node
{
public:
    PublishRefereeData();

    void publishGameStatus(const GameStatusPackage::data & pkg);
    void publishGameRobotHp(const GameRobotHpPackage::data & pkg);
    void publishEventData(const EventDataPackage::data & pkg);
    void publishRobotStatus(const RobotStatusPackage::data & pkg);
    void publishRobotPos(const RobotPosPackage::data & pkg);
    void publishHurtData(const HurtDataPackage::data & pkg);
    void publishRfidStatus(const RfidStatusPackage::data & pkg);
    void publishGroundRobotPosition(const GroundRobotPositionPackage::data & pkg);
    void publishSentryGimbal(float yaw, float pitch, float yaw_diff);
    void publishSentryInfo(const SentryInfoPackage::data & pkg);

private:
    // Publisher
    rclcpp::Publisher<combat_rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
    rclcpp::Publisher<combat_rm_interfaces::msg::GameRobotHp>::SharedPtr game_robot_hp_pub_;
    rclcpp::Publisher<combat_rm_interfaces::msg::EventData>::SharedPtr event_data_pub_;
    rclcpp::Publisher<combat_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
    rclcpp::Publisher<combat_rm_interfaces::msg::RobotPos>::SharedPtr robot_pos_pub_;
    rclcpp::Publisher<combat_rm_interfaces::msg::HurtData>::SharedPtr hurt_data_pub_;
    rclcpp::Publisher<combat_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_status_pub_;
    rclcpp::Publisher<combat_rm_interfaces::msg::GroundRobotPosition>::SharedPtr ground_robot_position_pub_;
    rclcpp::Publisher<combat_rm_interfaces::msg::SentryGimbal>::SharedPtr sentry_gimbal_pub_;
    rclcpp::Publisher<combat_rm_interfaces::msg::SentryInfo>::SharedPtr sentry_info_pub_;

    combat_rm_interfaces::msg::GameStatus game_status_msg_;
    combat_rm_interfaces::msg::GameRobotHp game_robot_hp_msg_;
    combat_rm_interfaces::msg::EventData event_data_msg_;
    combat_rm_interfaces::msg::RobotStatus robot_status_msg_;
    combat_rm_interfaces::msg::RobotPos robot_pos_msg_;
    combat_rm_interfaces::msg::HurtData hurt_data_msg_;
    combat_rm_interfaces::msg::RfidStatus rfid_status_msg_;
    combat_rm_interfaces::msg::GroundRobotPosition ground_robot_position_msg_;
    combat_rm_interfaces::msg::SentryGimbal sentry_gimbal_msg_;
    combat_rm_interfaces::msg::SentryInfo sentry_info_msg_;
};

}  // namespace io
