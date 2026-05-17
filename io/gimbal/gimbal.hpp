#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "serial/serial.h"
#include "io/packet_typedef.hpp"

namespace io
{

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  OUTPOST,     // 前哨站
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float yaw_diff;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  GameStatusPackage::data game_status() const;
  EventDataPackage::data event_data() const;
  RobotStatusPackage::data robot_status() const;
  HurtDataPackage::data hurt_data() const;
  SentryInfoPackage::data sentry_info() const;
  RfidStatusPackage::data rfid_status() const;
  RobotPosPackage::data robot_pos() const;
  GroundRobotPositionPackage::data ground_robot_pos() const;
  GameRobotHpPackage::data game_robot_hp() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);
  void send(
    uint8_t target_mode, uint8_t chassis_status, uint8_t sentry_status, float vx,
    float vy);
  void send(
    uint8_t target_mode, uint8_t chassis_status, uint8_t sentry_status, float vx, float vy,
    float vyaw, uint8_t terrain_status, uint8_t bump_status);

  void send(io::VisionToGimbal vision_to_gimbal);
  void send(io::NavToGimbal nav_to_gimbal);

private:
  serial::Serial serial_;
  mutable std::mutex serial_mutex_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;
  mutable std::mutex write_warn_mutex_;
  bool reconnecting_ = false;
  std::chrono::steady_clock::time_point last_write_warn_time_{
    std::chrono::steady_clock::time_point::min()};
  mutable std::mutex q_mutex_;
  mutable std::condition_variable q_condition_;

  GimbalToVision rx_data_;
  RefereePackage1 referee_package1_;
  RefereePackage2 referee_package2_;
  VisionToGimbal tx_data_v_;
  NavToGimbal tx_data_n_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  using QuaternionSample = std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>;
  static constexpr std::size_t MAX_Q_HISTORY_SIZE = 1000;
  std::deque<QuaternionSample> q_history_;

  bool read(uint8_t * buffer, size_t size);
  bool write_raw(const uint8_t * data, size_t size);
  void log_write_warn_throttled(const std::string & msg);
  void read_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
