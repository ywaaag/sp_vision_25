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
  
// 视觉包
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {0x5A, 0x01};
  float DWT_stamp;  // DWT计数器，单位为微秒
  
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  // 这里的enemycolor应该忽略，其值为yaml写死的0或1
  
  float q[4];    // wxyz顺序
  float pitch;
  float pitch_vel;
  float yaw;
  float yaw_vel;
  float yaw_diff;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数

  uint8_t reserved[12];
  uint8_t tail = 0x55;
  uint16_t check_sum;
};

// 自瞄下行包
struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {0x5A, 0x01};
  uint64_t time_stamp;
  
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float pitch;
  float pitch_vel;
  float pitch_acc;
  float yaw;
  float yaw_vel;
  float yaw_acc;

  uint8_t reserved[26];
  uint16_t check_sum;
  uint8_t tail = 0x55;
};

// 导航下行包
struct __attribute__((packed)) NavToGimbal
{
  uint8_t head[2] = {0x5A, 0x02};
  uint64_t time_stamp;
    
  uint8_t chassis_status;
  uint8_t sentry_status;
  uint8_t mode;  //0：对装甲板 1：对前哨站 2：对能量机关 3：对基地（）

  float vx;
  float vy;
  float vyaw;

  uint8_t reserved[36];
  uint16_t check_sum;
  uint8_t tail = 0x55;
};

// 裁判系统包
struct RefereePackage1
{
  uint8_t head[2];
  float DWT_stamp;  // DWT计数器，单位为微秒

  GameStatusPackage::data game_status_data;
  EventDataPackage::data event_data;
  RobotStatusPackage::data robot_status_data;
  HurtDataPackage::data hurt_data;
  SentryInfoPackage::data sentry_info_data;
  RfidStatusPackage::data rfid_status_data;

  uint8_t reserved[35];
  uint8_t tail;
  uint16_t check_sum;
} __attribute__((packed));

// 裁判系统包
struct RefereePackage2
{
  uint8_t head[2];
  float DWT_stamp;  // DWT计数器，单位为微秒

  RobotPosPackage::data robot_pos_data;
  GroundRobotPositionPackage::data ground_robot_pos_data;
  GameRobotHpPackage::data game_robot_hp_data;

  uint8_t reserved[3];
  uint8_t tail;
  uint16_t check_sum;
} __attribute__((packed));

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
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
    uint8_t mode, uint8_t chassis_status, uint8_t sentry_status, float vx,
    float vy);

  void send(io::VisionToGimbal VisionToGimbal);
  void send(io::NavToGimbal NavToGimbal);

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
