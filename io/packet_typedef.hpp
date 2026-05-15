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
const uint8_t SOF_VISION_HEAD = 0x5A;
const uint8_t SOF_SEND = 0x5A;
const uint8_t SOF_TAIL = 0x55;

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
