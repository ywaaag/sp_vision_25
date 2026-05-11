#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "armor.hpp"

namespace auto_aim
{
class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  void solve(Armor & armor) const;

  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  double outpost_reprojection_error(Armor armor, const double & pitch);

  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  void optimize_yaw(Armor & armor) const;
  void solve_ippe(Armor & armor, const std::vector<cv::Point3f> & object_points) const;
  void update_pose(Armor & armor, const cv::Vec3d & rvec, const cv::Vec3d & tvec) const;

  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
  double ippe_selection_cost(
    const Armor & armor, const std::vector<cv::Point3f> & object_points, const cv::Vec3d & rvec,
    const cv::Vec3d & tvec) const;
  double pose_reprojection_error(
    const std::vector<cv::Point3f> & object_points, const std::vector<cv::Point2f> & image_points,
    const cv::Vec3d & rvec, const cv::Vec3d & tvec) const;
  double lightbar_tilt_angle(const std::vector<cv::Point2f> & image_points) const;
  bool is_left_tilt_from_lightbar(double lightbar_tilt, ArmorName name) const;
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP
