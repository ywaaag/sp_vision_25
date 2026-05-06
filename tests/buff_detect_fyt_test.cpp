#include <fmt/core.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <opencv2/opencv.hpp>

#include "tasks/auto_buff_fyt/buff_aimer.hpp"
#include "tasks/auto_buff_fyt/buff_detector.hpp"
#include "tasks/auto_buff_fyt/buff_solver.hpp"
#include "tasks/auto_buff_fyt/buff_target.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace
{
const std::string keys =
  "{help h usage ? |                         | 输出命令行参数说明 }"
  "{config-path c  | configs/standard3.yaml  | yaml配置文件的路径}"
  "{display d      | true                    | 是否显示检测窗口  }"
  "{start-index s  | 0                       | 视频起始帧下标    }"
  "{end-index e    | 0                       | 视频结束帧下标    }"
  "{@video-path    | assets/big.avi         | 待检测avi路径     }";

void draw_rune_object(cv::Mat & img, const auto_buff_fyt::RuneObject & obj, bool selected_candidate)
{
  const std::vector<cv::Point2f> pts = obj.pts.to_vector2f();
  const auto poly_i = std::vector<cv::Point>{
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
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto video_path = cli.get<std::string>(0);
  const auto config_path = cli.get<std::string>("config-path");
  const auto display = cli.get<bool>("display");
  const auto start_index = cli.get<int>("start-index");
  const auto end_index = cli.get<int>("end-index");

  tools::Exiter exiter;
  auto_buff_fyt::Buff_Detector detector(config_path);
  auto_buff_fyt::Solver solver(config_path);
  auto_buff_fyt::BigTarget target;
  auto_buff_fyt::Aimer aimer(config_path);

  cv::VideoCapture video(video_path);
  if (!video.isOpened()) {
    tools::logger()->error("[buff_detect_fyt_test] failed to open video: {}", video_path);
    return 1;
  }

  std::filesystem::path text_path(video_path);
  text_path.replace_extension(".txt");
  std::ifstream text(text_path);
  const bool has_pose = text.is_open();
  auto t0 = std::chrono::steady_clock::now();

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  if (has_pose) {
    for (int i = 0; i < start_index; ++i) {
      double t, w, x, y, z;
      text >> t >> w >> x >> y >> z;
    }
  }

  int total_frames = 0;
  int model_detected_frames = 0;
  int selected_target_frames = 0;
  int solved_frames = 0;

  for (int frame_count = start_index; !exiter.exit(); ++frame_count) {
    if (end_index > 0 && frame_count > end_index) break;

    cv::Mat img;
    video.read(img);
    if (img.empty()) break;
    ++total_frames;

    std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now();
    if (has_pose) {
      double t, w, x, y, z;
      if (!(text >> t >> w >> x >> y >> z)) break;
      timestamp = t0 + std::chrono::microseconds(static_cast<int>(t * 1e6));
      solver.set_R_gimbal2world({w, x, y, z});
    }

    auto selected = detector.detect(img);
    const auto & filtered_objects = detector.last_filtered_objects();
    const auto & binary_roi = detector.last_binary_roi();

    if (!filtered_objects.empty()) ++model_detected_frames;
    if (selected.has_value()) ++selected_target_frames;

    for (const auto & obj : filtered_objects) {
      const bool is_selected_candidate =
        selected.has_value() && obj.type == auto_buff_fyt::RuneType::INACTIVATED &&
        cv::norm(obj.pts.bottom_left - selected->target().points[0]) < 1.0;
      draw_rune_object(img, obj, is_selected_candidate);
    }

    if (selected.has_value()) {
      auto & rune = selected.value();
      auto & fanblade = rune.target();
      for (int i = 0; i < 4; ++i) tools::draw_point(img, fanblade.points[i], {0, 255, 0}, 3);
      tools::draw_point(img, fanblade.center, {0, 0, 255}, 4);
      tools::draw_point(img, rune.r_center, {255, 255, 0}, 4);
    }

    if (has_pose) {
      solver.solve(selected);
      target.get_target(selected, timestamp);
      auto target_copy = target;
      auto command = aimer.aim(target_copy, timestamp, 22, false);
      if (!target.is_unsolve()) {
        ++solved_frames;
        const auto buff_center = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
        auto image_points =
          solver.reproject_buff(buff_center, target.ekf_x()[4], target.ekf_x()[5]);
        if (image_points.size() >= 4) {
          tools::draw_points(
            img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4),
            {0, 255, 255});
        }
        if (image_points.size() >= 5) tools::draw_point(img, image_points[4], {255, 0, 0}, 4);
        if (image_points.size() >= 6) tools::draw_point(img, image_points[5], {255, 255, 255}, 4);
      }
      if (command.control) {
        cv::putText(
          img,
          fmt::format("cmd {:.2f} {:.2f}", command.yaw * 57.3, command.pitch * 57.3),
          {20, 100}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
      }
    }

    if (!binary_roi.empty() && binary_roi.cols > 1 && binary_roi.rows > 1) {
      const cv::Rect roi(
        img.cols - binary_roi.cols, 0, binary_roi.cols,
        std::min(binary_roi.rows, img.rows));
      binary_roi(cv::Rect(0, 0, roi.width, roi.height)).copyTo(img(roi));
      cv::rectangle(img, roi, cv::Scalar(180, 180, 180), 2);
    }

    cv::putText(
      img,
      fmt::format(
        "frame:{} model:{}/{} target:{} solved:{}", frame_count, model_detected_frames,
        total_frames, selected_target_frames, solved_frames),
      {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

    if (display) {
      cv::imshow("buff_detect_fyt_test", img);
      const int key = cv::waitKey(30);
      if (key == 'q') break;
    }
  }

  if (display) cv::destroyAllWindows();

  tools::logger()->info(
    "[buff_detect_fyt_test] model detected {}/{} frames, selected {}/{} frames, solved {}/{} frames",
    model_detected_frames, total_frames, selected_target_frames, total_frames, solved_frames,
    total_frames);

  if (total_frames == 0) {
    tools::logger()->error("[buff_detect_fyt_test] no frames were read from {}", video_path);
    return 1;
  }

  return model_detected_frames > 0 ? 0 : 1;
}
