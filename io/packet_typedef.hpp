// Copyright 2026 Combat Robotics Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IO__PACKET_TYPEDEF_HPP_
#define IO__PACKET_TYPEDEF_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace io
{
const uint8_t SOF_REFREE_HEAD = 0xA5;
const uint8_t SOF_HEAD = 0x5A;
const uint8_t SOF_TAIL = 0x55;
const uint8_t RECEIVE_VISION_ID = 0x01;
const uint8_t RECEIVE_REFEREE1_ID = 0x02;
const uint8_t RECEIVE_REFEREE2_ID = 0x03;
const uint8_t SEND_NAV_ID = 0x55;

const uint8_t SOF_VISION_HEAD = SOF_HEAD;
const uint8_t SOF_SEND = SOF_HEAD;
const uint16_t PACKAGE_LENGTH = 64;
const float RECEIVE_TIMEOUT = 0.0f;

struct HeaderFrame
{
  uint8_t sof;   // 数据帧起始字节，固定值为 0xA5
  uint16_t len;  // 数据段长度
  uint8_t seq;   // 数据包序号
  uint8_t crc8;  // 数据帧头的 CRC8 校验
} __attribute__((packed));

/********************************************************/
/* Receive data                                         */
/********************************************************/

// 比赛状态数据包
struct GameStatusPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct data
  {
    uint8_t game_progress;
    uint16_t stage_remain_time;
  } __attribute__((packed));

  uint16_t crc16;
} __attribute__((packed));

// 场地事件数据包
struct EventDataPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct data
  {
    uint32_t ally_supply_zone_non_exchange : 1;
    uint32_t ally_supply_zone_exchange : 1;
    uint32_t ally_supply_zone : 1;

    uint32_t ally_small_power_rune : 2;
    uint32_t ally_big_power_rune : 2;

    uint32_t central_highland : 2;
    uint32_t trapezoidal_highland : 2;

    uint32_t reserved1 : 12;

    uint32_t center_gain_point : 2;
    uint32_t ally_fortress_gain_point : 2;
    uint32_t ally_outpost_gain_point : 2;
    uint32_t base_gain_point : 1;

    uint32_t reserved2 : 2;
  } __attribute__((packed));

  uint16_t crc16;
} __attribute__((packed));

// 比赛友方机器人血量数据包
struct GameRobotHpPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct data
  {
    uint16_t hero_hp;
    uint16_t engineer_hp;
    uint16_t standard_3_hp;
    uint16_t standard_4_hp;
    // uint16_t ally_7_robot_hp;
    uint16_t ally_outpost_hp;
    uint16_t ally_base_hp;
  } __attribute__((packed));

  uint16_t crc16;
} __attribute__((packed));

// 机器人位姿数据包
struct RobotPosPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct data
  {
    float x;
    float y;
  } __attribute__((packed));

  uint16_t crc16;
} __attribute__((packed));

// 机器人性能体系状态数据包
struct RobotStatusPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct data
  {
    uint16_t current_hp;
    uint16_t maximum_hp;
  } __attribute__((packed));

  uint16_t crc16;
} __attribute__((packed));

// 伤害状态数据包
struct HurtDataPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct data
  {
    uint8_t armor_id;
    uint8_t hp_deduction_reason;
  } __attribute__((packed));

  uint16_t crc16;
} __attribute__((packed));

// RFID 状态数据包
struct RfidStatusPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct data
  {
    uint32_t ally_base_gain_point : 1;
    uint32_t ally_central_highland_gain_point : 1;
    uint32_t enemy_central_highland_gain_point : 1;
    uint32_t reserved1 : 14;
    uint32_t ally_fortress_gain_point : 1;
    uint32_t ally_outpost_gain_point : 1;
    uint32_t ally_supply_point_non_exchange : 1;
    uint32_t ally_supply_point_exchange : 1;
    uint32_t reserved2 : 2;
    uint32_t center_gain_point : 1;
    uint32_t enemy_fortress_gain_point : 1;
    uint32_t enemy_outpost_gain_point : 1;
    uint32_t reserved3 : 6;
  } __attribute__((packed));

  uint8_t reserve4;
  uint16_t crc16;
} __attribute__((packed));

// 己方地面机器人位置数据包
struct GroundRobotPositionPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct data
  {
    float hero_x;
    float hero_y;

    float engineer_x;
    float engineer_y;

    float standard_3_x;
    float standard_3_y;

    float standard_4_x;
    float standard_4_y;

  } __attribute__((packed)) ;

  uint16_t crc16;
} __attribute__((packed));

// 哨兵信息数据包
struct SentryInfoPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct data
  {
    uint8_t disengaged_state;
    uint8_t current_state;
    uint8_t ally_power_rune_state;
    uint16_t projectile_allowance_17mm;
  } __attribute__((packed)) ;

  uint16_t crc16;
} __attribute__((packed));

/********************************************************/
/* Serial packages                                      */
/********************************************************/

// 视觉上行包
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {SOF_HEAD, RECEIVE_VISION_ID};
  float DWT_stamp;  // DWT计数器，单位为微秒

  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float pitch;
  float pitch_vel;
  float yaw;
  float yaw_vel;
  float yaw_diff;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数

  uint8_t reserved[12];
  uint8_t tail = SOF_TAIL;
  uint16_t check_sum;
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

  uint8_t reserved[33];
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

// 自瞄下行包
struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {SOF_HEAD, RECEIVE_VISION_ID};
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
  uint8_t tail = SOF_TAIL;
};

// 导航下行包
struct __attribute__((packed)) NavToGimbal
{
  uint8_t head[2] = {SOF_HEAD, RECEIVE_REFEREE1_ID};
  uint64_t time_stamp;

  uint8_t chassis_status;
  uint8_t sentry_status;
  uint8_t target_mode;  // scanmode: 0 空闲, 1 自瞄装甲板, 2 前哨站, 3 小符, 4 大符

  float vx;
  float vy;
  float vyaw;

  uint8_t terrain_status;
  uint8_t bump_status;

  uint8_t reserved[34];
  uint16_t check_sum;
  uint8_t tail = SOF_TAIL;
};

static_assert(sizeof(GimbalToVision) == PACKAGE_LENGTH, "GimbalToVision must be 64 bytes");
static_assert(sizeof(RefereePackage1) == PACKAGE_LENGTH, "RefereePackage1 must be 64 bytes");
static_assert(sizeof(RefereePackage2) == PACKAGE_LENGTH, "RefereePackage2 must be 64 bytes");
static_assert(sizeof(VisionToGimbal) == PACKAGE_LENGTH, "VisionToGimbal must be 64 bytes");
static_assert(sizeof(NavToGimbal) == PACKAGE_LENGTH, "NavToGimbal must be 64 bytes");

/********************************************************/
/* template                                             */
/********************************************************/

template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

template <typename T>
inline T fromVector(const std::vector<uint8_t> & data, size_t offset)
{
  T packet;
  const uint8_t * src = data.data() + offset;
  std::copy(src, src + sizeof(T), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T & data)
{
  std::vector<uint8_t> packet(sizeof(T));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
    packet.begin());
  return packet;
}

}  // namespace io

#endif  // IO__PACKET_TYPEDEF_HPP_
