#pragma once
#include <memory>

#include "combat_rm_interfaces/msg/target.hpp"
#include "rclcpp/rclcpp.hpp"
#include "publishrefereedata.hpp"
#include "subscribenavcmd.hpp"

namespace io {

class ROS2
{
public:
    ROS2();
    ~ROS2();

    void publish(const GameStatusPackage::data & pkg);
    void publish(const EventDataPackage::data & pkg);
    void publish(const RobotStatusPackage::data & pkg);
    void publish(const HurtDataPackage::data & pkg);
    void publish(const SentryInfoPackage::data & pkg);
    void publish(const RfidStatusPackage::data & pkg);
    void publish(const RobotPosPackage::data & pkg);
    void publish(const GroundRobotPositionPackage::data & pkg);
    void publish(const GameRobotHpPackage::data & pkg);
    void publish(const combat_rm_interfaces::msg::Target & msg);
    void publish(float yaw, float pitch, float yaw_diff);

    uint8_t getChassisStatus();
    uint8_t getSentryStatus();
    uint8_t getTargetMode();
    uint8_t getTerrainStatus();
    uint8_t getBumpStatus();
    float getCmdVelX();
    float getCmdVelY();
    float getCmdVelZ();

    void spin_some();

private:
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    bool owns_rclcpp_ = false;

    std::shared_ptr<PublishRefereeData> publishrefeedata_;
    std::shared_ptr<SubscribeNavCmd> subscribenavcmd_;
};

}  // namespace io
