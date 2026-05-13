#include "tasks/auto_aim/planner/planner.hpp"

#include <cmath>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Plan Planner::plan(auto_aim_ekfpnp::Target target, double bullet_speed)
{
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 22.5;
  }

  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  target.predict(bullet_traj.fly_time);

  double yaw0;
  Trajectory traj;
  try {
    yaw0 = aim(target, bullet_speed)(0);
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }

  return plan_trajectory(traj, yaw0);
}

Plan Planner::plan(std::optional<auto_aim_ekfpnp::Target> target, double bullet_speed)
{
  if (!target.has_value()) return {false};

  double decision_speed;
  double high_speed_delay_time;
  double low_speed_delay_time;
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    decision_speed = decision_speed_;
    high_speed_delay_time = high_speed_delay_time_;
    low_speed_delay_time = low_speed_delay_time_;
  }

  double delay_time = std::abs(target->ekf_x()[7]) > decision_speed ? high_speed_delay_time
                                                                    : low_speed_delay_time;

  auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));
  target->predict(future);

  return plan(*target, bullet_speed);
}

Eigen::Matrix<double, 2, 1> Planner::aim(
  const auto_aim_ekfpnp::Target & target, double bullet_speed)
{
  Eigen::Vector3d xyz;
  double yaw;
  auto min_dist = 1e10;

  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      yaw = xyza[3];
    }
  }
  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

  auto azim = std::atan2(xyz.y(), xyz.x());
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  double yaw_offset;
  double pitch_offset;
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    yaw_offset = yaw_offset_;
    pitch_offset = pitch_offset_;
  }

  return {tools::limit_rad(azim + yaw_offset), -bullet_traj.pitch - pitch_offset};
}

Trajectory Planner::get_trajectory(
  auto_aim_ekfpnp::Target & target, double yaw0, double bullet_speed)
{
  Trajectory traj;

  target.predict(-DT * (HALF_HORIZON + 1));
  auto yaw_pitch_last = aim(target, bullet_speed);

  target.predict(DT);
  auto yaw_pitch = aim(target, bullet_speed);

  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    auto yaw_pitch_next = aim(target, bullet_speed);

    auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return traj;
}

}  // namespace auto_aim
