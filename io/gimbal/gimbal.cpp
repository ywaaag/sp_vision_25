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

  {
    std::unique_lock<std::mutex> lock(q_mutex_);
    q_condition_.wait(lock, [this] { return !q_history_.empty(); });
  }
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  q_condition_.notify_all();
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
    case GimbalMode::OUTPOST:
      return "OUTPOST";
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
  auto interpolate = [t](const QuaternionSample & sample_a, const QuaternionSample & sample_b) {
    const auto & [q_a, t_a] = sample_a;
    const auto & [q_b, t_b] = sample_b;
    auto t_ab = tools::delta_time(t_a, t_b);
    if (t_ab == 0.0) return q_b;

    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    return q_a.slerp(k, q_b).normalized();
  };

  std::unique_lock<std::mutex> lock(q_mutex_);
  q_condition_.wait(lock, [this] { return q_history_.size() >= 2 || quit_; });

  while (!quit_ && std::get<1>(q_history_.back()) < t) {
    q_condition_.wait(lock, [this, &t] {
      return quit_ || (q_history_.size() >= 2 && std::get<1>(q_history_.back()) >= t);
    });
  }

  if (q_history_.empty()) return Eigen::Quaterniond::Identity();
  if (q_history_.size() == 1) return std::get<0>(q_history_.front());

  if (t <= std::get<1>(q_history_.front())) return interpolate(q_history_[0], q_history_[1]);

  for (std::size_t i = 1; i < q_history_.size(); ++i) {
    if (t <= std::get<1>(q_history_[i])) {
      return interpolate(q_history_[i - 1], q_history_[i]);
    }
  }

  return interpolate(q_history_[q_history_.size() - 2], q_history_[q_history_.size() - 1]);
}

/**
 * @brief 
 * 
 * @param vision_to_gimbal
 * @note 发送自瞄下行包
 * 
 */

void Gimbal::send(io::VisionToGimbal vision_to_gimbal)
{
  tx_data_v_ = VisionToGimbal{};
  tx_data_v_.mode = vision_to_gimbal.mode;
  tx_data_v_.yaw = vision_to_gimbal.yaw;
  tx_data_v_.yaw_vel = vision_to_gimbal.yaw_vel;
  tx_data_v_.yaw_acc = vision_to_gimbal.yaw_acc;
  tx_data_v_.pitch = vision_to_gimbal.pitch;
  tx_data_v_.pitch_vel = vision_to_gimbal.pitch_vel;
  tx_data_v_.pitch_acc = vision_to_gimbal.pitch_acc;
  tx_data_v_.time_stamp = vision_to_gimbal.time_stamp;

  tools::append_check_sum(
    reinterpret_cast<uint8_t *>(&tx_data_v_), sizeof(tx_data_v_));

  write_raw(reinterpret_cast<uint8_t *>(&tx_data_v_), sizeof(tx_data_v_));
}

/**
 * @brief 
 * 
 * @param nav_to_gimbal
 * @note 发送导航下行包
 * 
 */

void Gimbal::send(io::NavToGimbal nav_to_gimbal)
{
  tx_data_n_ = NavToGimbal{};
  tx_data_n_.target_mode = nav_to_gimbal.target_mode;
  tx_data_n_.chassis_status = nav_to_gimbal.chassis_status;
  tx_data_n_.sentry_status = nav_to_gimbal.sentry_status;
  tx_data_n_.vx = nav_to_gimbal.vx;
  tx_data_n_.vy = nav_to_gimbal.vy;
  tx_data_n_.vyaw = nav_to_gimbal.vyaw;
  tx_data_n_.terrain_status = nav_to_gimbal.terrain_status;
  tx_data_n_.bump_status = nav_to_gimbal.bump_status;
  tx_data_n_.time_stamp = nav_to_gimbal.time_stamp;

  tools::append_check_sum(
    reinterpret_cast<uint8_t *>(&tx_data_n_), sizeof(tx_data_n_));

  write_raw(reinterpret_cast<uint8_t *>(&tx_data_n_), sizeof(tx_data_n_));
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
  tx_data_v_ = VisionToGimbal{};
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

  write_raw(reinterpret_cast<uint8_t *>(&tx_data_v_), sizeof(tx_data_v_));
}

/**
 * @brief 
 * 
 * @param target_mode
 * @param chassis_status 
 * @param sentry_status 
 * @param vx 
 * @param vy 
 * 
 * @note 发送导航下行包
 */

void Gimbal::send(
  uint8_t target_mode, uint8_t chassis_status, uint8_t sentry_status, float vx,
  float vy)
{
  send(target_mode, chassis_status, sentry_status, vx, vy, 0.0f, 0, 0);
}

void Gimbal::send(
  uint8_t target_mode, uint8_t chassis_status, uint8_t sentry_status, float vx, float vy,
  float vyaw, uint8_t terrain_status, uint8_t bump_status)
{
  tx_data_n_ = NavToGimbal{};
  tx_data_n_.target_mode = target_mode;
  tx_data_n_.chassis_status = chassis_status;
  tx_data_n_.sentry_status = sentry_status;
  tx_data_n_.vx = vx;
  tx_data_n_.vy = vy;
  tx_data_n_.vyaw = vyaw;
  tx_data_n_.terrain_status = terrain_status;
  tx_data_n_.bump_status = bump_status;
  tx_data_n_.time_stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count();
  
  tools::append_check_sum(
    reinterpret_cast<uint8_t *>(&tx_data_n_), sizeof(tx_data_n_));
  
  write_raw(reinterpret_cast<uint8_t *>(&tx_data_n_), sizeof(tx_data_n_));

}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (reconnecting_ || !serial_.isOpen()) return false;
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

bool Gimbal::write_raw(const uint8_t * data, size_t size)
{
  try {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (reconnecting_ || !serial_.isOpen()) {
      log_write_warn_throttled("[Gimbal] Serial not ready, command dropped.");
      return false;
    }
    serial_.write(data, size);
    return true;
  } catch (const std::exception & e) {
    log_write_warn_throttled(std::string("[Gimbal] Failed to write serial: ") + e.what());
    return false;
  }
}

void Gimbal::log_write_warn_throttled(const std::string & msg)
{
  auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(write_warn_mutex_);
  if (now - last_write_warn_time_ < std::chrono::milliseconds(500)) return;
  last_write_warn_time_ = now;
  tools::logger()->warn("{}", msg);
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  const auto reconnect_timeout = std::chrono::seconds(2);
  auto first_error_time = std::chrono::steady_clock::time_point::min();

  auto mark_error = [&]() {
    auto now = std::chrono::steady_clock::now();
    if (first_error_time == std::chrono::steady_clock::time_point::min()) {
      first_error_time = now;
      return;
    }

    if (now - first_error_time >= reconnect_timeout) {
      tools::logger()->warn("[Gimbal] Serial read abnormal for too long, attempting to reconnect...");
      reconnect();
      first_error_time = std::chrono::steady_clock::time_point::min();
    }
  };

  auto clear_error = [&]() { first_error_time = std::chrono::steady_clock::time_point::min(); };

  while (!quit_) {
    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      mark_error();
      continue;
    }

    if (rx_data_.head[0] != SOF_HEAD) {
      mark_error();
      continue;
    }

    switch (rx_data_.head[1]) {
      case RECEIVE_VISION_ID:  {
        auto t = std::chrono::steady_clock::now();

        if (!read(
              reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
              sizeof(rx_data_) - sizeof(rx_data_.head))) {
          mark_error();
          continue;
        }

        if (rx_data_.tail != SOF_TAIL) {
          tools::logger()->warn("[Gimbal] Invalid tail: {:02X}", rx_data_.tail);
          mark_error();
          continue;
        }

        if (!tools::verify_check_sum16(
              reinterpret_cast<uint8_t *>(&rx_data_),
              sizeof(rx_data_))) {
          tools::logger()->warn("[Gimbal] Invalid check sum");
          mark_error();
          continue;
        }

        clear_error();
        Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
        {
          std::lock_guard<std::mutex> lock(q_mutex_);
          q_history_.push_back({q, t});
          if (q_history_.size() > MAX_Q_HISTORY_SIZE) q_history_.pop_front();
        }
        q_condition_.notify_all();

        std::lock_guard<std::mutex> lock(mutex_);

        state_.yaw = rx_data_.yaw;
        state_.yaw_vel = rx_data_.yaw_vel;
        state_.yaw_diff = rx_data_.yaw_diff;
        state_.pitch = -rx_data_.pitch;
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
            mode_ = GimbalMode::OUTPOST;
            break;
          case 3:
            mode_ = GimbalMode::SMALL_BUFF;
            break;
          case 4:
            mode_ = GimbalMode::BIG_BUFF;
            break;
          default:
            mode_ = GimbalMode::IDLE;
            tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
            break;
        }
        
        break;
      }
      case RECEIVE_REFEREE1_ID:  {
        referee_package1_.head[0] = SOF_HEAD;
        referee_package1_.head[1] = RECEIVE_REFEREE1_ID;

        if (!read(
              reinterpret_cast<uint8_t *>(&referee_package1_) + sizeof(referee_package1_.head),
              sizeof(referee_package1_) - sizeof(referee_package1_.head))) {
          mark_error();
          continue;
        }

        if (referee_package1_.tail != SOF_TAIL) {
          tools::logger()->warn("[Gimbal] Invalid tail in referee package 1: {:02X}", referee_package1_.tail);
          mark_error();
          continue;
        }

        if (!tools::verify_check_sum16(
              reinterpret_cast<uint8_t *>(&referee_package1_),
              sizeof(referee_package1_))) {
          tools::logger()->warn("[Gimbal] Invalid check sum in referee package 1");
          mark_error();
          continue;
        }

        clear_error();
        break;
      }
      case RECEIVE_REFEREE2_ID: {
        referee_package2_.head[0] = SOF_HEAD;
        referee_package2_.head[1] = RECEIVE_REFEREE2_ID;

        if (!read(
              reinterpret_cast<uint8_t *>(&referee_package2_) + sizeof(referee_package2_.head),
              sizeof(referee_package2_) - sizeof(referee_package2_.head))) {
          mark_error();
          continue;
        }

        if (referee_package2_.tail != SOF_TAIL) {
          tools::logger()->warn("[Gimbal] Invalid tail in referee package 2: {:02X}", referee_package2_.tail);
          mark_error();
          continue;
        }

        if (!tools::verify_check_sum16(
              reinterpret_cast<uint8_t *>(&referee_package2_),
              sizeof(referee_package2_))) {
          tools::logger()->warn("[Gimbal] Invalid check sum in referee package 2");
          mark_error();
          continue;
        }

        clear_error();
        break;
      }
      default:  {
        tools::logger()->warn("[Gimbal] Invalid package head");
        mark_error();
        continue;
      }
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    reconnecting_ = true;
  }

  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_.isOpen()) serial_.close();
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_.open();  // 尝试重新打开
      }
      {
        std::lock_guard<std::mutex> lock(q_mutex_);
        q_history_.clear();
      }
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    reconnecting_ = false;
  }
}

}  // namespace io
