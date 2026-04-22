#include <fmt/format.h>

#include <numeric>
#include <string>
#include <vector>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_buff_fyt/buff_aimer.hpp"
#include "tasks/auto_buff_fyt/buff_detector.hpp"
#include "tasks/auto_buff_fyt/buff_solver.hpp"
#include "tasks/auto_buff_fyt/buff_target.hpp"
#include "tasks/auto_buff_fyt/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/trajectory.hpp"

namespace
{
const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | yaml配置文件路径 }";

void draw_rune_object(
  cv::Mat & img, const auto_buff_fyt::RuneObject & obj, bool selected_candidate)
{
  const std::vector<cv::Point2f> pts = obj.pts.to_vector2f();
  const std::vector<cv::Point> poly_i = {
    cv::Point(obj.pts.r_center),
    cv::Point(obj.pts.bottom_left),
    cv::Point(obj.pts.top_left),
    cv::Point(obj.pts.top_right),
    cv::Point(obj.pts.bottom_right)};
  const cv::Point2f aim_point =
    std::accumulate(pts.begin() + 1, pts.end(), cv::Point2f(0.0f, 0.0f)) / 4.0f;

  const auto line_color =
    obj.type == auto_buff_fyt::RuneType::INACTIVATED ? cv::Scalar(50, 255, 50) :
                                                       cv::Scalar(255, 50, 255);
  const int thickness = selected_candidate ? 3 : 2;
  cv::putText(
    img, fmt::format("{:.2f}", obj.prob), cv::Point2i(pts[1]), cv::FONT_HERSHEY_SIMPLEX, 0.8,
    line_color, 2);
  cv::polylines(img, poly_i, true, line_color, thickness);
  cv::circle(img, aim_point, 5, line_color, -1);

  const std::string rune_type =
    obj.type == auto_buff_fyt::RuneType::INACTIVATED ? "_HIT" : "_OK";
  const std::string rune_color = auto_buff_fyt::enemy_color_to_string(obj.color);
  cv::putText(
    img, rune_color + rune_type, cv::Point2i(pts[2]), cv::FONT_HERSHEY_SIMPLEX, 0.8, line_color,
    2);
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Plotter plotter;
  tools::Recorder recorder;
  tools::Exiter exiter;

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_buff_fyt::Buff_Detector detector(config_path);
  auto_buff_fyt::Solver solver(config_path);
  // auto_buff_fyt::SmallTarget target;
  auto_buff_fyt::BigTarget target;
  auto_buff_fyt::Aimer aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    q = gimbal.q(t);
    auto gs = gimbal.state();
    // recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);

    auto power_runes = detector.detect(img);
    const auto & filtered_objects = detector.last_filtered_objects();
    const auto & binary_roi = detector.last_binary_roi();

    solver.solve(power_runes);

    target.get_target(power_runes, t);

    auto target_copy = target;

    auto plan = aimer.mpc_aim(target_copy, t, gs, true);

    gimbal.send(
      plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
      plan.pitch_acc);

    nlohmann::json data;

    if (power_runes.has_value()) {
      const auto & p = power_runes.value();
      data["buff_R_yaw"] = p.ypd_in_world[0];
      data["buff_R_pitch"] = p.ypd_in_world[1];
      data["buff_R_dis"] = p.ypd_in_world[2];
      data["buff_yaw"] = p.ypr_in_world[0] * 57.3;
      data["buff_pitch"] = p.ypr_in_world[1] * 57.3;
      data["buff_roll"] = p.ypr_in_world[2] * 57.3;
    }

    for (const auto & obj : filtered_objects) {
      const bool is_selected_candidate =
        std::find_if(
          filtered_objects.begin(), filtered_objects.end(), [&](const auto & filtered) {
            return filtered.type == obj.type && filtered.color == obj.color &&
                   std::abs(filtered.prob - obj.prob) < 1e-3 &&
                   cv::norm(filtered.pts.bottom_left - obj.pts.bottom_left) < 1.0;
          }) != filtered_objects.end();
      draw_rune_object(img, obj, is_selected_candidate);
    }

    if (!target.is_unsolve()) {
      auto & p = power_runes.value();

      for (int i = 0; i < 4; ++i) tools::draw_point(img, p.target().points[i]);
      tools::draw_point(img, p.target().center, {0, 0, 255}, 3);
      tools::draw_point(img, p.r_center, {0, 0, 255}, 3);

      auto Rxyz_in_world_now = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
      auto image_points =
        solver.reproject_buff(Rxyz_in_world_now, target.ekf_x()[4], target.ekf_x()[5]);
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4), {0, 255, 0});
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin() + 4, image_points.end()), {0, 255, 0});

      auto Rxyz_in_world_pre = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
      image_points =
        solver.reproject_buff(Rxyz_in_world_pre, target_copy.ekf_x()[4], target_copy.ekf_x()[5]);
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4), {255, 0, 0});
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin() + 4, image_points.end()), {255, 0, 0});

      Eigen::VectorXd x = target.ekf_x();
      data["R_yaw"] = x[0];
      data["R_V_yaw"] = x[1];
      data["R_pitch"] = x[2];
      data["R_dis"] = x[3];
      data["yaw"] = x[4] * 57.3;
      data["angle"] = x[5] * 57.3;
      data["spd"] = x[6] * 57.3;
      if (x.size() >= 10) {
        data["spd"] = x[6];
        data["a"] = x[7];
        data["w"] = x[8];
        data["fi"] = x[9];
        data["spd0"] = target.spd;
      }
    }

    if (!binary_roi.empty() && binary_roi.cols > 1 && binary_roi.rows > 1) {
      const cv::Rect roi(
        img.cols - binary_roi.cols, 0, binary_roi.cols,
        std::min(binary_roi.rows, img.rows));
      binary_roi(cv::Rect(0, 0, roi.width, roi.height)).copyTo(img(roi));
      cv::rectangle(img, roi, cv::Scalar(180, 180, 180), 2);
    }

    data["gimbal_yaw"] = gs.yaw * 57.3;
    data["gimbal_pitch"] = gs.pitch * 57.3;
    data["gimbal_yaw_vel"] = gs.yaw_vel * 57.3;
    data["gimbal_pitch_vel"] = gs.pitch_vel * 57.3;

    if (plan.control) {
      data["plan_yaw"] = plan.yaw * 57.3;
      data["plan_pitch"] = plan.pitch * 57.3;
      data["plan_yaw_vel"] = plan.yaw_vel * 57.3;
      data["plan_pitch_vel"] = plan.pitch_vel * 57.3;
      data["plan_yaw_acc"] = plan.yaw_acc * 57.3;
      data["plan_pitch_acc"] = plan.pitch_acc * 57.3;
      data["shoot"] = plan.fire ? 1 : 0;
    }

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("result", img);

    const auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}
