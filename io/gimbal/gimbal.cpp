#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include "tools/check_sum.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point> first;
  if (queue_.pop(first)) {
    tools::logger()->info("[Gimbal] First q received.");
  } else {
    tools::logger()->warn("[Gimbal] Queue closed before first q received.");
  }
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

GameStatusPackage::data Gimbal::game_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return referee_package1_.game_status_data;
}

EventDataPackage::data Gimbal::event_data() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return referee_package1_.event_data;
}

RobotStatusPackage::data Gimbal::robot_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return referee_package1_.robot_status_data;
}

HurtDataPackage::data Gimbal::hurt_data() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return referee_package1_.hurt_data;
}

SentryInfoPackage::data Gimbal::sentry_info() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return referee_package1_.sentry_info_data;
}

RfidStatusPackage::data Gimbal::rfid_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return referee_package1_.rfid_status_data;
}

RobotPosPackage::data Gimbal::robot_pos() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return referee_package2_.robot_pos_data;
}

GroundRobotPositionPackage::data Gimbal::ground_robot_pos() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return referee_package2_.ground_robot_pos_data;
}

GameRobotHpPackage::data Gimbal::game_robot_hp() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return referee_package2_.game_robot_hp_data;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point> sample_a;
    if (!queue_.pop(sample_a)) {
      return Eigen::Quaterniond::Identity();
    }

    std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point> sample_b;
    if (!queue_.front(sample_b)) {
      return std::get<0>(sample_a);
    }

    const auto & [q_a, t_a] = sample_a;
    const auto & [q_b, t_b] = sample_b;
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

/**
 * @brief 
 * 
 * @param VisionToGimbal 
 * @note 发送自瞄下行包
 * 
 */

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_v_.mode = VisionToGimbal.mode;
  tx_data_v_.yaw = VisionToGimbal.yaw;
  tx_data_v_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_v_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_v_.pitch = VisionToGimbal.pitch;
  tx_data_v_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_v_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_v_.time_stamp = VisionToGimbal.time_stamp;

  tools::append_check_sum(
    reinterpret_cast<uint8_t *>(&tx_data_v_), sizeof(tx_data_v_));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_v_), sizeof(tx_data_v_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

/**
 * @brief 
 * 
 * @param NavToGimbal 
 * @note 发送导航下行包
 * 
 */

void Gimbal::send(io::NavToGimbal NavToGimbal)
{
  tx_data_n_.mode = NavToGimbal.mode;
  tx_data_n_.chassis_status = NavToGimbal.chassis_status;
  tx_data_n_.sentry_status = NavToGimbal.sentry_status;
  tx_data_n_.vx = NavToGimbal.vx;
  tx_data_n_.vy = NavToGimbal.vy;
  tx_data_n_.vyaw = NavToGimbal.vyaw;
  tx_data_n_.time_stamp = NavToGimbal.time_stamp;

  tools::append_check_sum(
    reinterpret_cast<uint8_t *>(&tx_data_n_), sizeof(tx_data_n_));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_n_), sizeof(tx_data_n_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

/**
 * @brief 
 * 
 * @param control 
 * @param fire 
 * @param yaw 
 * @param yaw_vel 
 * @param yaw_acc 
 * @param pitch 
 * @param pitch_vel 
 * @param pitch_acc 
 * 
 * @note 发送自瞄下行包
 */

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_v_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_v_.pitch = pitch;
  tx_data_v_.pitch_vel = pitch_vel;
  tx_data_v_.pitch_acc = pitch_acc;
  tx_data_v_.yaw = yaw;
  tx_data_v_.yaw_vel = yaw_vel;
  tx_data_v_.yaw_acc = yaw_acc;
  tx_data_v_.time_stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count();

  tools::append_check_sum(
    reinterpret_cast<uint8_t *>(&tx_data_v_), sizeof(tx_data_v_));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_v_), sizeof(tx_data_v_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

/**
 * @brief 
 * 
 * @param mode 
 * @param chassis_status 
 * @param sentry_status 
 * @param vx 
 * @param vy 
 * 
 * @note 发送导航下行包
 */

void Gimbal::send(
  uint8_t mode, uint8_t chassis_status, uint8_t sentry_status, float vx,
  float vy)
{
  tx_data_n_.mode = mode;
  tx_data_n_.chassis_status = chassis_status;
  tx_data_n_.sentry_status = sentry_status;
  tx_data_n_.vx = vx;
  tx_data_n_.vy = vy;
  tx_data_n_.vyaw = 0.0f;  // 导航包中暂时不使用yaw，填0
  tx_data_n_.time_stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count();
  
  tools::append_check_sum(
    reinterpret_cast<uint8_t *>(&tx_data_n_), sizeof(tx_data_n_));
  
  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_n_), sizeof(tx_data_n_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }

}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (rx_data_.head[0] != 0x5A) continue;

    switch (rx_data_.head[1]) {
      case 0x01:  {
        auto t = std::chrono::steady_clock::now();

        if (!read(
              reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
              sizeof(rx_data_) - sizeof(rx_data_.head))) {
          error_count++;
          continue;
        }

        if (rx_data_.tail != 0x55) {
          tools::logger()->warn("[Gimbal] Invalid tail: {:02X}", rx_data_.tail);
          error_count++;
          continue;
        }

        if (!tools::verify_check_sum16(
              reinterpret_cast<uint8_t *>(&rx_data_),
              sizeof(rx_data_))) {
          tools::logger()->warn("[Gimbal] Invalid check sum");
          error_count++;
          continue;
        }

        error_count = 0;
        Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
        queue_.push({q, t});

        std::lock_guard<std::mutex> lock(mutex_);

        state_.yaw = rx_data_.yaw;
        state_.yaw_vel = rx_data_.yaw_vel;
        state_.pitch = rx_data_.pitch;
        state_.pitch_vel = rx_data_.pitch_vel;
        state_.bullet_speed = rx_data_.bullet_speed;
        state_.bullet_count = rx_data_.bullet_count;

        switch (rx_data_.mode) {
          case 0:
            mode_ = GimbalMode::IDLE;
            break;
          case 1:
            mode_ = GimbalMode::AUTO_AIM;
            break;
          case 2:
            mode_ = GimbalMode::SMALL_BUFF;
            break;
          case 3:
            mode_ = GimbalMode::BIG_BUFF;
            break;
          default:
            mode_ = GimbalMode::IDLE;
            tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
            break;
        }
        
        break;
      }
      case 0x02:  {
        referee_package1_.head[0] = 0x5A;
        referee_package1_.head[1] = 0x02;

        if (!read(
              reinterpret_cast<uint8_t *>(&referee_package1_) + sizeof(referee_package1_.head),
              sizeof(referee_package1_) - sizeof(referee_package1_.head))) {
          error_count++;
          continue;
        }

        if (referee_package1_.tail != 0x55) {
          tools::logger()->warn("[Gimbal] Invalid tail in referee package 1: {:02X}", referee_package1_.tail);
          error_count++;
          continue;
        }

        if (!tools::verify_check_sum16(
              reinterpret_cast<uint8_t *>(&referee_package1_),
              sizeof(referee_package1_))) {
          tools::logger()->warn("[Gimbal] Invalid check sum in referee package 1");
          error_count++;
          continue;
        }

        break;
      }
      case 0x03: {
        referee_package2_.head[0] = 0x5A;
        referee_package2_.head[1] = 0x03;

        if (!read(
              reinterpret_cast<uint8_t *>(&referee_package2_) + sizeof(referee_package2_.head),
              sizeof(referee_package2_) - sizeof(referee_package2_.head))) {
          error_count++;
          continue;
        }

        if (referee_package2_.tail != 0x55) {
          tools::logger()->warn("[Gimbal] Invalid tail in referee package 2: {:02X}", referee_package2_.tail);
          error_count++;
          continue;
        }

        if (!tools::verify_check_sum16(
              reinterpret_cast<uint8_t *>(&referee_package2_),
              sizeof(referee_package2_))) {
          tools::logger()->warn("[Gimbal] Invalid check sum in referee package 2");
          error_count++;
          continue;
        }

        break;
      }
      default:  {
        tools::logger()->warn("[Gimbal] Invalid package head");
        error_count++;
        continue;
      }
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io
