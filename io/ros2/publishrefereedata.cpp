#include "publishrefereedata.hpp"

namespace io
{

PublishRefereeData::PublishRefereeData() : Node("referee_publisher")
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort();

  game_status_pub_ =
    create_publisher<combat_rm_interfaces::msg::GameStatus>("referee/game_status", qos);
  game_robot_hp_pub_ =
    create_publisher<combat_rm_interfaces::msg::GameRobotHp>("referee/game_robot_hp", qos);
  event_data_pub_ =
    create_publisher<combat_rm_interfaces::msg::EventData>("referee/event_data", qos);
  robot_status_pub_ =
    create_publisher<combat_rm_interfaces::msg::RobotStatus>("referee/robot_status", qos);
  robot_pos_pub_ = create_publisher<combat_rm_interfaces::msg::RobotPos>("referee/robot_pos", qos);
  hurt_data_pub_ = create_publisher<combat_rm_interfaces::msg::HurtData>("referee/hurt_data", qos);
  rfid_status_pub_ =
    create_publisher<combat_rm_interfaces::msg::RfidStatus>("referee/rfid_status", qos);
  ground_robot_position_pub_ = create_publisher<combat_rm_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", qos);
  armors_pub_ = create_publisher<combat_rm_interfaces::msg::Armors>(
    "armor_detector/armors", rclcpp::SensorDataQoS());
  sentry_gimbal_pub_ =
    create_publisher<combat_rm_interfaces::msg::SentryGimbal>("/sentry_gimbal", qos);
  sentry_info_pub_ =
    create_publisher<combat_rm_interfaces::msg::SentryInfo>("referee/sentry_info", qos);

  RCLCPP_INFO(get_logger(), "referee_publisher initialized.");
}

void PublishRefereeData::publishGameStatus(const GameStatusPackage::data & pkg)
{
  game_status_msg_.game_progress = pkg.game_progress;
  game_status_msg_.stage_remain_time = pkg.stage_remain_time;
  game_status_pub_->publish(game_status_msg_);
}

void PublishRefereeData::publishGameRobotHp(const GameRobotHpPackage::data & pkg)
{
  game_robot_hp_msg_.hero_hp = pkg.hero_hp;
  game_robot_hp_msg_.engineer_hp = pkg.engineer_hp;
  game_robot_hp_msg_.standard_3_hp = pkg.standard_3_hp;
  game_robot_hp_msg_.standard_4_hp = pkg.standard_4_hp;
  game_robot_hp_msg_.ally_outpost_hp = pkg.ally_outpost_hp;
  game_robot_hp_msg_.ally_base_hp = pkg.ally_base_hp;
  game_robot_hp_pub_->publish(game_robot_hp_msg_);
}

void PublishRefereeData::publishEventData(const EventDataPackage::data & pkg)
{
  event_data_msg_.ally_supply_zone_non_exchange = pkg.ally_supply_zone_non_exchange;
  event_data_msg_.ally_supply_zone_exchange = pkg.ally_supply_zone_exchange;
  event_data_msg_.ally_supply_zone = pkg.ally_supply_zone;
  event_data_msg_.ally_small_power_rune = pkg.ally_small_power_rune;
  event_data_msg_.ally_big_power_rune = pkg.ally_big_power_rune;
  event_data_msg_.central_highland = pkg.central_highland;
  event_data_msg_.trapezoidal_highland = pkg.trapezoidal_highland;
  event_data_msg_.center_gain_point = pkg.center_gain_point;
  event_data_msg_.ally_fortress_gain_point = pkg.ally_fortress_gain_point;
  event_data_msg_.ally_outpost_gain_point = pkg.ally_outpost_gain_point;
  event_data_msg_.base_gain_point = pkg.base_gain_point;

  event_data_pub_->publish(event_data_msg_);
}

void PublishRefereeData::publishRobotStatus(const RobotStatusPackage::data & pkg)
{
  robot_status_msg_.current_hp = pkg.current_hp;
  robot_status_msg_.maximum_hp = pkg.maximum_hp;
  robot_status_pub_->publish(robot_status_msg_);
}

void PublishRefereeData::publishRobotPos(const RobotPosPackage::data & pkg)
{
  robot_pos_msg_.x = pkg.x;
  robot_pos_msg_.y = pkg.y;
  robot_pos_pub_->publish(robot_pos_msg_);
}

void PublishRefereeData::publishHurtData(const HurtDataPackage::data & pkg)
{
  hurt_data_msg_.armor_id = pkg.armor_id;
  hurt_data_msg_.hp_deduction_reason = pkg.hp_deduction_reason;
  hurt_data_pub_->publish(hurt_data_msg_);
}

void PublishRefereeData::publishRfidStatus(const RfidStatusPackage::data & pkg)
{
  rfid_status_msg_.ally_base_gain_point = pkg.ally_base_gain_point;
  rfid_status_msg_.ally_central_highland_gain_point = pkg.ally_central_highland_gain_point;
  rfid_status_msg_.enemy_central_highland_gain_point = pkg.enemy_central_highland_gain_point;
  rfid_status_msg_.ally_fortress_gain_point = pkg.ally_fortress_gain_point;
  rfid_status_msg_.ally_outpost_gain_point = pkg.ally_outpost_gain_point;
  rfid_status_msg_.ally_supply_point_non_exchange = pkg.ally_supply_point_non_exchange;
  rfid_status_msg_.ally_supply_point_exchange = pkg.ally_supply_point_exchange;
  rfid_status_msg_.center_gain_point = pkg.center_gain_point;
  rfid_status_msg_.enemy_fortress_gain_point = pkg.enemy_fortress_gain_point;
  rfid_status_msg_.enemy_outpost_gain_point = pkg.enemy_outpost_gain_point;

  rfid_status_pub_->publish(rfid_status_msg_);
}

void PublishRefereeData::publishGroundRobotPosition(const GroundRobotPositionPackage::data & pkg)
{
  ground_robot_position_msg_.hero_position.x = pkg.hero_x;
  ground_robot_position_msg_.hero_position.y = pkg.hero_y;
  ground_robot_position_msg_.engineer_position.x = pkg.engineer_x;
  ground_robot_position_msg_.engineer_position.y = pkg.engineer_y;
  ground_robot_position_msg_.standard_3_position.x = pkg.standard_3_x;
  ground_robot_position_msg_.standard_3_position.y = pkg.standard_3_y;
  ground_robot_position_msg_.standard_4_position.x = pkg.standard_4_x;
  ground_robot_position_msg_.standard_4_position.y = pkg.standard_4_y;

  ground_robot_position_pub_->publish(ground_robot_position_msg_);
}

void PublishRefereeData::publishArmors(const combat_rm_interfaces::msg::Armors & msg)
{
  armors_pub_->publish(msg);
}

void PublishRefereeData::publishSentryGimbal(float yaw, float pitch, float yaw_diff)
{
  sentry_gimbal_msg_.yaw = yaw;
  sentry_gimbal_msg_.pitch = pitch;
  sentry_gimbal_msg_.yaw_diff = yaw_diff;

  sentry_gimbal_pub_->publish(sentry_gimbal_msg_);
}

void PublishRefereeData::publishSentryInfo(const SentryInfoPackage::data & pkg)
{
  sentry_info_msg_.disengaged_state = pkg.disengaged_state;
  sentry_info_msg_.current_state = pkg.current_state;
  sentry_info_msg_.ally_power_rune_state = pkg.ally_power_rune_state;
  sentry_info_msg_.projectile_allowance_17mm = pkg.projectile_allowance_17mm;

  sentry_info_pub_->publish(sentry_info_msg_);
}

}  // namespace io
