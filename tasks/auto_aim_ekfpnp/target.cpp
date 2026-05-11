#include "target.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim_ekfpnp
{
Target::Target(
  const auto_aim::Armor & armor, std::chrono::steady_clock::time_point t, double radius,
  int armor_num, Eigen::VectorXd P0_dig, const auto_aim::Solver & solver)
: name(armor.name),
  armor_type(armor.type),
  priority(armor.priority),
  jumped(false),
  last_id(0),
  armor_num_(armor_num),
  switch_count_(0),
  update_count_(0),
  is_switch_(false),
  is_converged_(false),
  outpost_height_ready_(false),
  outpost_observed_mask_(0),
  outpost_height_sums_{0.0, 0.0, 0.0},
  outpost_height_counts_{0, 0, 0},
  outpost_height_order_{0, 0, 0},
  solver_(&solver),
  t_(t)
{
  const auto r = radius;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  const double center_x = xyz[0] + r * std::cos(ypr[0]);
  const double center_y = xyz[1] + r * std::sin(ypr[0]);
  const double center_z = xyz[2];

  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0}};
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
  const auto dt = tools::delta_time(t, t_);
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

  double v1 = 100.0;
  double v2 = 400.0;
  if (name == auto_aim::ArmorName::outpost) {
    v1 = 10.0;
    v2 = 0.1;
  }

  const double a = dt * dt * dt * dt / 4.0;
  const double b = dt * dt * dt / 2.0;
  const double c = dt * dt;

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

  if (convergened() && name == auto_aim::ArmorName::outpost && std::abs(ekf_.x[7]) > 2.0) {
    ekf_.x[7] = ekf_.x[7] > 0 ? 2.51 : -2.51;
  }

  ekf_.predict(F, Q, f);
}

void Target::update(const auto_aim::Armor & armor)
{
  int id = 0;
  double min_angle_error = 1e10;
  const auto xyza_list = armor_xyza_list();

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  xyza_i_list.reserve(armor_num_);
  for (int i = 0; i < armor_num_; ++i) {
    xyza_i_list.push_back({xyza_list[i], i});
  }

  std::sort(
    xyza_i_list.begin(), xyza_i_list.end(),
    [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
      Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
      Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
      return ypd1[2] < ypd2[2];
    });

  for (int i = 0; i < std::min(3, static_cast<int>(xyza_i_list.size())); ++i) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
    double angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                         std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));
    if (use_outpost_staggered_height_model()) {
      angle_error += std::abs(tools::limit_rad(armor.ypd_in_world[1] - ypd[1]));
    }

    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }

  if (id != 0) jumped = true;

  is_switch_ = id != last_id;
  if (is_switch_) ++switch_count_;

  last_id = id;
  ++update_count_;

  record_outpost_observation(id, armor);
  update_pixel_points(armor, id);
}

void Target::update_pixel_points(const auto_aim::Armor & armor, int id)
{
  Eigen::MatrixXd H = measurement_jacobian(ekf_.x, id);
  Eigen::MatrixXd R = measurement_covariance(ekf_.x);

  auto h = [&](const Eigen::VectorXd & x) {
    return measurement_function(x, id);
  };

  Eigen::Matrix<double, PIXEL_OBSERVATION_DIM, 1> z = measurement_vector(armor.points);
  ekf_.update(z, H, R, h);
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> armor_xyza;
  armor_xyza.reserve(armor_num_);
  for (int i = 0; i < armor_num_; ++i) {
    const double angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    armor_xyza.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return armor_xyza;
}

bool Target::diverged() const
{
  const bool r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  const bool l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

bool Target::convergened()
{
  if (name != auto_aim::ArmorName::outpost && update_count_ > 3 && !diverged()) {
    is_converged_ = true;
  }

  if (name == auto_aim::ArmorName::outpost && update_count_ > 10 && !diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

bool Target::is_outpost_target() const
{
  return name == auto_aim::ArmorName::outpost && armor_num_ == OUTPOST_ARMOR_COUNT;
}

bool Target::use_outpost_staggered_height_model() const
{
  return is_outpost_target() && outpost_height_ready_;
}

void Target::record_outpost_observation(int id, const auto_aim::Armor & armor)
{
  if (!is_outpost_target() || id < 0 || id >= OUTPOST_ARMOR_COUNT) return;

  outpost_observed_mask_ |= (1 << id);
  outpost_height_sums_[id] += armor.xyz_in_world[2];
  outpost_height_counts_[id] += 1;

  if (outpost_height_ready_ || outpost_observed_mask_ != ((1 << OUTPOST_ARMOR_COUNT) - 1)) {
    return;
  }

  std::array<std::pair<double, int>, OUTPOST_ARMOR_COUNT> avg_height_and_id{};
  for (int i = 0; i < OUTPOST_ARMOR_COUNT; ++i) {
    const double avg_z = outpost_height_sums_[i] / std::max(1, outpost_height_counts_[i]);
    avg_height_and_id[i] = {avg_z, i};
  }
  std::sort(
    avg_height_and_id.begin(), avg_height_and_id.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.first < rhs.first; });

  outpost_height_order_.fill(0);
  outpost_height_order_[avg_height_and_id[0].second] = -1;
  outpost_height_order_[avg_height_and_id[1].second] = 0;
  outpost_height_order_[avg_height_and_id[2].second] = 1;
  outpost_height_ready_ = true;

  ekf_.x[4] = avg_height_and_id[1].first;
  tools::logger()->info(
    "[Target] Outpost staggered height model activated, middle z = {:.3f}",
    avg_height_and_id[1].first);
}

double Target::outpost_height_offset(int id) const
{
  if (!use_outpost_staggered_height_model() || id < 0 || id >= OUTPOST_ARMOR_COUNT) return 0.0;
  return static_cast<double>(outpost_height_order_[id]) * OUTPOST_ARMOR_HEIGHT_STEP;
}

Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  const double angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  const bool use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  const double r = use_l_h ? x[8] + x[9] : x[8];
  const double armor_x = x[0] - r * std::cos(angle);
  const double armor_y = x[2] - r * std::sin(angle);
  const double armor_z = use_l_h ? x[4] + x[10] : x[4] + outpost_height_offset(id);

  return {armor_x, armor_y, armor_z};
}

Eigen::Matrix<double, Target::PIXEL_OBSERVATION_DIM, 1> Target::measurement_vector(
  const std::vector<cv::Point2f> & image_points) const
{
  Eigen::Matrix<double, PIXEL_OBSERVATION_DIM, 1> z;
  z.setZero();

  if (image_points.size() != 4) return z;

  for (int i = 0; i < 4; ++i) {
    z[2 * i] = image_points[i].x;
    z[2 * i + 1] = image_points[i].y;
  }
  return z;
}

Eigen::Matrix<double, Target::PIXEL_OBSERVATION_DIM, 1> Target::measurement_function(
  const Eigen::VectorXd & x, int id) const
{
  Eigen::Matrix<double, PIXEL_OBSERVATION_DIM, 1> z;
  z.setZero();

  if (solver_ == nullptr) return z;

  const double yaw = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  const Eigen::Vector3d xyz = h_armor_xyz(x, id);
  const auto image_points = solver_->reproject_armor(xyz, yaw, armor_type, name);
  return measurement_vector(image_points);
}

Eigen::MatrixXd Target::measurement_jacobian(const Eigen::VectorXd & x, int id) const
{
  Eigen::MatrixXd H(PIXEL_OBSERVATION_DIM, x.size());
  H.setZero();

  for (int col = 0; col < x.size(); ++col) {
    const double eps = std::max(JACOBIAN_EPS, std::abs(x[col]) * JACOBIAN_EPS);
    Eigen::VectorXd x_plus = x;
    Eigen::VectorXd x_minus = x;
    x_plus[col] += eps;
    x_minus[col] -= eps;
    x_plus[6] = tools::limit_rad(x_plus[6]);
    x_minus[6] = tools::limit_rad(x_minus[6]);

    const auto z_plus = measurement_function(x_plus, id);
    const auto z_minus = measurement_function(x_minus, id);
    H.col(col) = (z_plus - z_minus) / (2.0 * eps);
  }

  return H;
}

Eigen::MatrixXd Target::measurement_covariance(const Eigen::VectorXd & x) const
{
  const double center_distance =
    std::max(Eigen::Vector3d(x[0], x[2], x[4]).norm(), 1e-6);
  const double pixel_variance =
    std::max(PIXEL_R_AT_1M * std::log(1.0 / center_distance + 1.0), PIXEL_R_MIN);

  return Eigen::MatrixXd::Identity(PIXEL_OBSERVATION_DIM, PIXEL_OBSERVATION_DIM) * pixel_variance;
}

}  // namespace auto_aim_ekfpnp
