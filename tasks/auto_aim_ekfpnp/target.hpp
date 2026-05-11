#ifndef AUTO_AIM_EKFPNP__TARGET_HPP
#define AUTO_AIM_EKFPNP__TARGET_HPP

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim_ekfpnp
{
class Target
{
public:
  auto_aim::ArmorName name;
  auto_aim::ArmorType armor_type;
  auto_aim::ArmorPriority priority;
  bool jumped;
  int last_id;

  Target() = default;
  Target(
    const auto_aim::Armor & armor, std::chrono::steady_clock::time_point t, double radius,
    int armor_num, Eigen::VectorXd P0_dig, const auto_aim::Solver & solver);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const auto_aim::Armor & armor);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;
  bool convergened();

private:
  static constexpr double OUTPOST_ARMOR_HEIGHT_STEP = 0.1;
  static constexpr int OUTPOST_ARMOR_COUNT = 3;
  static constexpr int PIXEL_OBSERVATION_DIM = 8;
  static constexpr double PIXEL_R_AT_1M = 70.0;
  static constexpr double PIXEL_R_MIN = 30.0;
  static constexpr double JACOBIAN_EPS = 1e-4;

  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_;
  bool is_converged_;
  bool outpost_height_ready_;
  int outpost_observed_mask_;
  std::array<double, OUTPOST_ARMOR_COUNT> outpost_height_sums_;
  std::array<int, OUTPOST_ARMOR_COUNT> outpost_height_counts_;
  std::array<int, OUTPOST_ARMOR_COUNT> outpost_height_order_;

  const auto_aim::Solver * solver_;
  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  void update_pixel_points(const auto_aim::Armor & armor, int id);

  bool is_outpost_target() const;
  bool use_outpost_staggered_height_model() const;
  void record_outpost_observation(int id, const auto_aim::Armor & armor);
  double outpost_height_offset(int id) const;

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::Matrix<double, PIXEL_OBSERVATION_DIM, 1> measurement_vector(
    const std::vector<cv::Point2f> & image_points) const;
  Eigen::Matrix<double, PIXEL_OBSERVATION_DIM, 1> measurement_function(
    const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd measurement_jacobian(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd measurement_covariance(const Eigen::VectorXd & x) const;
};

}  // namespace auto_aim_ekfpnp

#endif  // AUTO_AIM_EKFPNP__TARGET_HPP
