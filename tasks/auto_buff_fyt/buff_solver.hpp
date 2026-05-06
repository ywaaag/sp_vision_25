#ifndef AUTO_BUFF_FYT__BUFF_SOLVER_HPP
#define AUTO_BUFF_FYT__BUFF_SOLVER_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <optional>
#include <vector>

#include "buff_type.hpp"
#include "tools/math_tools.hpp"

namespace auto_buff_fyt
{
class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  void solve(std::optional<PowerRune> & power_rune) const;

  cv::Point2f point_buff2pixel(cv::Point3f x);

  std::vector<cv::Point2f> reproject_buff(
    const Eigen::Vector3d & xyz_in_world, double yaw, double roll) const;

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  mutable cv::Vec3d rvec_;
  mutable cv::Vec3d tvec_;

  const std::vector<cv::Point3f> PNP_OBJECT_POINTS_ = {
    cv::Point3f(0, 186e-3f, 541.5e-3f),
    cv::Point3f(0, 160e-3f, 858.5e-3f),
    cv::Point3f(0, -160e-3f, 858.5e-3f),
    cv::Point3f(0, -186e-3f, 541.5e-3f),
    cv::Point3f(0, 0, 0)};

  const std::vector<cv::Point3f> DRAW_OBJECT_POINTS_ = {
    cv::Point3f(0, 186e-3f, 541.5e-3f),
    cv::Point3f(0, 160e-3f, 858.5e-3f),
    cv::Point3f(0, -160e-3f, 858.5e-3f),
    cv::Point3f(0, -186e-3f, 541.5e-3f),
    cv::Point3f(0, 0, 700e-3f),
    cv::Point3f(0, 0, 0)};
};
}  // namespace auto_buff_fyt

#endif  // AUTO_BUFF_FYT__BUFF_SOLVER_HPP
