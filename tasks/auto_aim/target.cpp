#include "target.hpp"

#include <algorithm>
#include <numeric>
#include <utility>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace
{
constexpr double OUTPOST_DIRECTION_READY_SPEED = 0.5;
}

Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  update_count_(0),
  armor_num_(armor_num),
  switch_count_(0),
  is_switch_(false),
  is_converged_(false),
  outpost_height_ready_(false),
  outpost_fixed_model_ready_(false),
  outpost_observed_mask_(0),
  outpost_height_sums_{0.0, 0.0, 0.0},
  outpost_height_counts_{0, 0, 0},
  outpost_height_order_{0, 0, 0},
  outpost_fixed_center_(Eigen::Vector3d::Zero()),
  outpost_fixed_radius_(0.0),
  outpost_fixed_w_(0.0),
  t_(t)
{
  auto r = radius;
  priority = armor.priority;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // Rotation center coordinates in world frame.
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1 for normal 4-armor targets
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
}

Target::Target(double x, double vyaw, double radius, double h)
: armor_num_(4),
  switch_count_(0),
  update_count_(0),
  is_switch_(false),
  is_converged_(false),
  outpost_height_ready_(false),
  outpost_fixed_model_ready_(false),
  outpost_observed_mask_(0),
  outpost_height_sums_{0.0, 0.0, 0.0},
  outpost_height_counts_{0, 0, 0},
  outpost_height_order_{0, 0, 0},
  outpost_fixed_center_(Eigen::Vector3d::Zero()),
  outpost_fixed_radius_(0.0),
  outpost_fixed_w_(0.0)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  double v1, v2;
  if (name == ArmorName::outpost) {
    v1 = 10;
    v2 = 0.1;
  } else {
    v1 = 100;
    v2 = 400;
  }
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2) {
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;
  }

  ekf_.predict(F, Q, f);
  apply_outpost_fixed_model();
}

void Target::update(const Armor & armor)
{
  int id = 0;
  auto min_angle_error = 1e10;
  const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++) {
    xyza_i_list.push_back({xyza_list[i], i});
  }

  std::sort(
    xyza_i_list.begin(), xyza_i_list.end(),
    [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
      Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
      Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
      return ypd1[2] < ypd2[2];
    });

  for (int i = 0; i < std::min(3, static_cast<int>(xyza_i_list.size())); i++) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
    auto association_score = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                             std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));
    if (use_outpost_staggered_height_model()) {
      const auto pitch_error = std::abs(tools::limit_rad(armor.ypd_in_world[1] - ypd[1]));
      const auto height_error =
        std::abs(armor.xyz_in_world[2] - xyza[2]) / OUTPOST_ARMOR_HEIGHT_STEP;
      association_score += 2.0 * pitch_error + height_error;
    }

    if (association_score < min_angle_error) {
      id = xyza_i_list[i].second;
      min_angle_error = association_score;
    }
  }

  if (id != 0) jumped = true;

  is_switch_ = id != last_id;
  if (is_switch_) switch_count_++;

  last_id = id;
  update_count_++;

  if (is_outpost_target()) {
    std::array<double, OUTPOST_ARMOR_COUNT> avg_z{};
    for (int i = 0; i < OUTPOST_ARMOR_COUNT; ++i) {
      avg_z[i] = outpost_height_counts_[i] > 0
                   ? outpost_height_sums_[i] / outpost_height_counts_[i]
                   : 0.0;
    }

    tools::logger()->info(
      "[OutpostDebug] update id={} switch={} ready={} mask={} obs_xyz=({:.3f},{:.3f},{:.3f}) "
      "obs_ypd=({:.3f},{:.3f},{:.3f}) obs_yaw={:.3f} ekf_center=({:.3f},{:.3f},{:.3f}) "
      "ekf_v=({:.3f},{:.3f},{:.3f}) a={:.3f} w={:.3f} r={:.4f} avg_z=[{:.3f},{:.3f},{:.3f}] "
      "cnt=[{},{},{}] order=[{},{},{}]",
      id, is_switch_ ? 1 : 0, outpost_height_ready_ ? 1 : 0, outpost_observed_mask_,
      armor.xyz_in_world[0], armor.xyz_in_world[1], armor.xyz_in_world[2], armor.ypd_in_world[0],
      armor.ypd_in_world[1], armor.ypd_in_world[2], armor.ypr_in_world[0], ekf_.x[0], ekf_.x[2],
      ekf_.x[4], ekf_.x[1], ekf_.x[3], ekf_.x[5], ekf_.x[6], ekf_.x[7], ekf_.x[8], avg_z[0],
      avg_z[1], avg_z[2], outpost_height_counts_[0], outpost_height_counts_[1],
      outpost_height_counts_[2], outpost_height_order_[0], outpost_height_order_[1],
      outpost_height_order_[2]);

    for (int i = 0; i < armor_num_; ++i) {
      const auto predicted_xyz = h_armor_xyz(ekf_.x, i);
      const auto predicted_ypd = tools::xyz2ypd(predicted_xyz);
      const auto predicted_angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
      tools::logger()->info(
        "[OutpostDebug] pred id={} xyz=({:.3f},{:.3f},{:.3f}) ypd=({:.3f},{:.3f},{:.3f}) "
        "angle={:.3f} dz_to_obs={:.3f}",
        i, predicted_xyz[0], predicted_xyz[1], predicted_xyz[2], predicted_ypd[0],
        predicted_ypd[1], predicted_ypd[2], predicted_angle,
        armor.xyz_in_world[2] - predicted_xyz[2]);
    }
  }

  record_outpost_observation(id, armor);
  update_ypda(armor, id);
  apply_outpost_fixed_model();
}

void Target::update_ypda(const Armor & armor, int id)
{
  const bool lock_outpost_height = use_outpost_staggered_height_model();
  const double outpost_center_z = ekf_.x[4];
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  if (lock_outpost_height) {
    H.row(1).setZero();
    H(2, 4) = 0.0;
  }

  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  Eigen::VectorXd R_dig{
    {4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1,
     log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}};
  if (lock_outpost_height) {
    R_dig[1] = 1e6;
  }

  Eigen::MatrixXd R = R_dig.asDiagonal();

  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};

  ekf_.update(z, H, R, h, z_subtract);
  if (lock_outpost_height) {
    ekf_.x[4] = outpost_center_z;
    ekf_.x[5] = 0.0;
  }
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

bool Target::diverged() const
{
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[9] + ekf_.x[8] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

bool Target::is_outpost_target() const
{
  return name == ArmorName::outpost && armor_num_ == OUTPOST_ARMOR_COUNT;
}

bool Target::use_outpost_staggered_height_model() const
{
  return is_outpost_target() && outpost_height_ready_;
}

void Target::record_outpost_observation(int id, const Armor & armor)
{
  if (!is_outpost_target() || id < 0 || id >= OUTPOST_ARMOR_COUNT) return;

  outpost_observed_mask_ |= (1 << id);
  outpost_height_sums_[id] += armor.xyz_in_world[2];
  outpost_height_counts_[id] += 1;

  tools::logger()->info(
    "[OutpostDebug] record id={} z={:.3f} sum={:.3f} count={} mask={} ready={}", id,
    armor.xyz_in_world[2], outpost_height_sums_[id], outpost_height_counts_[id],
    outpost_observed_mask_, outpost_height_ready_ ? 1 : 0);

  if (outpost_height_ready_ || outpost_observed_mask_ != ((1 << OUTPOST_ARMOR_COUNT) - 1)) {
    return;
  }

  std::array<double, OUTPOST_ARMOR_COUNT> avg_height{};
  for (int i = 0; i < OUTPOST_ARMOR_COUNT; ++i) {
    avg_height[i] = outpost_height_sums_[i] / std::max(1, outpost_height_counts_[i]);
  }

  if (std::abs(ekf_.x[7]) < OUTPOST_DIRECTION_READY_SPEED) {
    tools::logger()->info(
      "[OutpostDebug] height model pending direction w={:.3f} avg_z=[{:.3f},{:.3f},{:.3f}]",
      ekf_.x[7], avg_height[0], avg_height[1], avg_height[2]);
    return;
  }

  outpost_height_order_.fill(0);
  const auto high_it = std::max_element(avg_height.begin(), avg_height.end());
  const int high_id = static_cast<int>(std::distance(avg_height.begin(), high_it));
  const int next_id = (high_id + 1) % OUTPOST_ARMOR_COUNT;
  const int prev_id = (high_id + OUTPOST_ARMOR_COUNT - 1) % OUTPOST_ARMOR_COUNT;
  const bool positive_direction = ekf_.x[7] > 0;
  const int low_id = positive_direction ? prev_id : next_id;
  const int middle_id = positive_direction ? next_id : prev_id;
  outpost_height_order_[high_id] = 1;
  outpost_height_order_[low_id] = -1;
  outpost_height_order_[middle_id] = 0;
  outpost_height_ready_ = true;

  std::array<double, OUTPOST_ARMOR_COUNT> center_z_candidates{};
  for (int i = 0; i < OUTPOST_ARMOR_COUNT; ++i) {
    center_z_candidates[i] =
      avg_height[i] - static_cast<double>(outpost_height_order_[i]) * OUTPOST_ARMOR_HEIGHT_STEP;
  }
  std::sort(center_z_candidates.begin(), center_z_candidates.end());

  ekf_.x[4] = center_z_candidates[OUTPOST_ARMOR_COUNT / 2];
  ekf_.x[5] = 0.0;
  outpost_fixed_center_ = {ekf_.x[0], ekf_.x[2], ekf_.x[4]};
  outpost_fixed_radius_ = ekf_.x[8];
  outpost_fixed_w_ = positive_direction ? 2.51 : -2.51;
  outpost_fixed_model_ready_ = true;
  apply_outpost_fixed_model();
  tools::logger()->info(
    "[OutpostDebug] height model activated direction={} sequence={} high=(id {}, z {:.3f}) "
    "low=(id {}, z {:.3f}) mid=(id {}, z {:.3f}) order=[{},{},{}] fixed_center=({:.3f},{:.3f},{:.3f}) "
    "fixed_r={:.4f} fixed_w={:.3f}",
    positive_direction ? "positive" : "negative",
    positive_direction ? "high-low-middle" : "high-middle-low", high_id, avg_height[high_id],
    low_id, avg_height[low_id], middle_id, avg_height[middle_id], outpost_height_order_[0],
    outpost_height_order_[1], outpost_height_order_[2], outpost_fixed_center_[0],
    outpost_fixed_center_[1], outpost_fixed_center_[2], outpost_fixed_radius_, outpost_fixed_w_);
}

double Target::outpost_height_offset(int id) const
{
  if (!use_outpost_staggered_height_model() || id < 0 || id >= OUTPOST_ARMOR_COUNT) return 0.0;
  return static_cast<double>(outpost_height_order_[id]) * OUTPOST_ARMOR_HEIGHT_STEP;
}

void Target::apply_outpost_fixed_model()
{
  if (!outpost_fixed_model_ready_) return;

  ekf_.x[0] = outpost_fixed_center_[0];
  ekf_.x[1] = 0.0;
  ekf_.x[2] = outpost_fixed_center_[1];
  ekf_.x[3] = 0.0;
  ekf_.x[4] = outpost_fixed_center_[2];
  ekf_.x[5] = 0.0;
  ekf_.x[7] = outpost_fixed_w_;
  ekf_.x[8] = outpost_fixed_radius_;
  ekf_.x[9] = 0.0;
  ekf_.x[10] = 0.0;
}

Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = use_l_h ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = use_l_h ? x[4] + x[10] : x[4] + outpost_height_offset(id);

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = use_l_h ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = use_l_h ? -std::cos(angle) : 0.0;
  auto dy_dl = use_l_h ? -std::sin(angle) : 0.0;

  const double dz_dh = use_l_h ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim
