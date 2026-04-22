#include "buff_solver.hpp"

namespace auto_buff_fyt
{
Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  const Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

void Solver::solve(std::optional<PowerRune> & power_rune) const
{
  if (!power_rune.has_value()) return;

  auto & rune = power_rune.value();
  if (rune.target().points.size() < 4) {
    power_rune = std::nullopt;
    return;
  }

  std::vector<cv::Point2f> image_points = {
    rune.target().points[0],
    rune.target().points[1],
    rune.target().points[2],
    rune.target().points[3],
    rune.r_center};

  if (
    !cv::solvePnP(
      PNP_OBJECT_POINTS_, image_points, camera_matrix_, distort_coeffs_, rvec_, tvec_, false,
      cv::SOLVEPNP_ITERATIVE))
  {
    power_rune = std::nullopt;
    return;
  }

  Eigen::Vector3d t_buff2camera;
  cv::cv2eigen(tvec_, t_buff2camera);
  cv::Mat rmat;
  cv::Rodrigues(rvec_, rmat);
  Eigen::Matrix3d R_buff2camera;
  cv::cv2eigen(rmat, R_buff2camera);

  const Eigen::Vector3d blade_xyz_in_buff{0.0, 0.0, 700e-3};
  const Eigen::Vector3d xyz_in_camera = t_buff2camera;
  const Eigen::Vector3d blade_xyz_in_camera = R_buff2camera * blade_xyz_in_buff + t_buff2camera;

  const Eigen::Matrix3d R_buff2gimbal = R_camera2gimbal_ * R_buff2camera;
  const Eigen::Vector3d xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  const Eigen::Vector3d blade_xyz_in_gimbal =
    R_camera2gimbal_ * blade_xyz_in_camera + t_camera2gimbal_;

  const Eigen::Matrix3d R_buff2world = R_gimbal2world_ * R_buff2gimbal;

  rune.xyz_in_world = R_gimbal2world_ * xyz_in_gimbal;
  rune.ypd_in_world = tools::xyz2ypd(rune.xyz_in_world);
  rune.blade_xyz_in_world = R_gimbal2world_ * blade_xyz_in_gimbal;
  rune.blade_ypd_in_world = tools::xyz2ypd(rune.blade_xyz_in_world);
  rune.ypr_in_world = tools::eulers(R_buff2world, 2, 1, 0);
}

cv::Point2f Solver::point_buff2pixel(cv::Point3f x)
{
  std::vector<cv::Point3d> world_points = {x};
  std::vector<cv::Point2d> image_points;
  cv::projectPoints(world_points, rvec_, tvec_, camera_matrix_, distort_coeffs_, image_points);
  return image_points.back();
}

std::vector<cv::Point2f> Solver::reproject_buff(
  const Eigen::Vector3d & xyz_in_world, double yaw, double roll) const
{
  const auto R_buff2world = tools::rotation_matrix(Eigen::Vector3d(yaw, 0.0, roll));
  const Eigen::Matrix3d R_buff2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_buff2world;
  const Eigen::Vector3d t_buff2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * xyz_in_world - t_camera2gimbal_);

  cv::Vec3d rvec;
  cv::Mat R_buff2camera_cv;
  cv::eigen2cv(R_buff2camera, R_buff2camera_cv);
  cv::Rodrigues(R_buff2camera_cv, rvec);
  cv::Vec3d tvec(t_buff2camera[0], t_buff2camera[1], t_buff2camera[2]);

  std::vector<cv::Point2f> image_points;
  cv::projectPoints(DRAW_OBJECT_POINTS_, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}
}  // namespace auto_buff_fyt
