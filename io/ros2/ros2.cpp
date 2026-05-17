#include "ros2.hpp"

namespace io {

ROS2::ROS2()
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
        owns_rclcpp_ = true;
    }

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();

    publishrefeedata_ = std::make_shared<PublishRefereeData>();
    subscribenavcmd_ = std::make_shared<SubscribeNavCmd>();

    executor_->add_node(publishrefeedata_);
    executor_->add_node(subscribenavcmd_);

    RCLCPP_INFO(rclcpp::get_logger("ROS2"), "ROS2 initialized.");
}

ROS2::~ROS2()
{
    if (executor_) {
        if (publishrefeedata_) executor_->remove_node(publishrefeedata_);
        if (subscribenavcmd_) executor_->remove_node(subscribenavcmd_);
    }

    publishrefeedata_.reset();
    subscribenavcmd_.reset();
    executor_.reset();

    if (owns_rclcpp_ && rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

void ROS2::publish(const GameStatusPackage::data & pkg)
{
    publishrefeedata_->publishGameStatus(pkg);
}

void ROS2::publish(const EventDataPackage::data & pkg)
{
    publishrefeedata_->publishEventData(pkg);
}

void ROS2::publish(const RobotStatusPackage::data & pkg)
{
    publishrefeedata_->publishRobotStatus(pkg);
}

void ROS2::publish(const HurtDataPackage::data & pkg)
{
    publishrefeedata_->publishHurtData(pkg);
}

void ROS2::publish(const SentryInfoPackage::data & pkg)
{
    publishrefeedata_->publishSentryInfo(pkg);
}

void ROS2::publish(const RfidStatusPackage::data & pkg)
{
    publishrefeedata_->publishRfidStatus(pkg);
}

void ROS2::publish(const RobotPosPackage::data & pkg)
{
    publishrefeedata_->publishRobotPos(pkg);
}

void ROS2::publish(const GroundRobotPositionPackage::data & pkg)
{
    publishrefeedata_->publishGroundRobotPosition(pkg);
}

void ROS2::publish(const GameRobotHpPackage::data & pkg)
{
    publishrefeedata_->publishGameRobotHp(pkg);
}

void ROS2::publish(const combat_rm_interfaces::msg::Armors & msg)
{
    publishrefeedata_->publishArmors(msg);
}

void ROS2::publish(float yaw, float pitch, float yaw_diff)
{
    publishrefeedata_->publishSentryGimbal(yaw, pitch, yaw_diff);
}

uint8_t ROS2::getChassisStatus()
{
    return subscribenavcmd_->getChassisStatus();
}

uint8_t ROS2::getSentryStatus()
{
    return subscribenavcmd_->getSentryStatus();
}

uint8_t ROS2::getTargetMode()
{
    return subscribenavcmd_->getTargetMode();
}

uint8_t ROS2::getTerrainStatus()
{
    return subscribenavcmd_->getTerrainStatus();
}

uint8_t ROS2::getBumpStatus()
{
    return subscribenavcmd_->getBumpStatus();
}

float ROS2::getCmdVelX()
{
    return subscribenavcmd_->getCmdVelX();
}

float ROS2::getCmdVelY()
{
    return subscribenavcmd_->getCmdVelY();
}

float ROS2::getCmdVelZ()
{
    return subscribenavcmd_->getCmdVelZ();
}

void ROS2::spin_some()
{
    if (executor_) {
        executor_->spin_some();
    }
}

}  // namespace io
