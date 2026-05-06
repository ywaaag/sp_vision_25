#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace
{
const std::string keys =
  "{help h usage ? |                         | show help message}"
  "{config-path c  | configs/standard3.yaml  | path to yaml config}"
  "{display d      | true                    | display debug window}"
  "{start-index s  | 0                       | first frame index}"
  "{end-index e    | 0                       | last frame index, 0 means run to end}"
  "{@video-path    | assets/outpost1.avi     | input video path}";

struct VideoMetadata
{
  double fps;
  int width;
  int height;
  int total_frames;
  int wait_ms;
  double display_scale;
};

VideoMetadata read_video_metadata(cv::VideoCapture & video)
{
  double fps = video.get(cv::CAP_PROP_FPS);
  if (!std::isfinite(fps) || fps <= 1e-3) fps = 30.0;

  const int width = static_cast<int>(std::lround(video.get(cv::CAP_PROP_FRAME_WIDTH)));
  const int height = static_cast<int>(std::lround(video.get(cv::CAP_PROP_FRAME_HEIGHT)));
  const int total_frames = static_cast<int>(std::lround(video.get(cv::CAP_PROP_FRAME_COUNT)));

  constexpr double kMaxDisplayWidth = 1600.0;
  constexpr double kMaxDisplayHeight = 900.0;
  const double scale_x = width > 0 ? kMaxDisplayWidth / width : 1.0;
  const double scale_y = height > 0 ? kMaxDisplayHeight / height : 1.0;

  double display_scale = std::min({1.0, scale_x, scale_y});
  if (!std::isfinite(display_scale) || display_scale <= 0.0) display_scale = 1.0;

  return {
    fps,
    width,
    height,
    total_frames,
    std::max(1, static_cast<int>(std::lround(1000.0 / fps))),
    display_scale};
}

std::chrono::steady_clock::time_point frame_timestamp(
  const std::chrono::steady_clock::time_point & t0, int frame_index, double fps)
{
  return t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(static_cast<double>(frame_index) / fps));
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
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);

  cv::VideoCapture video(video_path);
  if (!video.isOpened()) {
    tools::logger()->error("[outpost_test] failed to open video: {}", video_path);
    return 1;
  }

  const auto metadata = read_video_metadata(video);
  tools::logger()->info(
    "[outpost_test] video={} fps={:.2f} size={}x{} frames={} wait={}ms scale={:.3f}",
    video_path, metadata.fps, metadata.width, metadata.height, metadata.total_frames,
    metadata.wait_ms, metadata.display_scale);

  solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  auto t0 = std::chrono::steady_clock::now();

  int total_frames = 0;
  int outpost_detected_frames = 0;
  int tracked_frames = 0;
  bool quit_requested = false;

  for (int frame_index = start_index; !exiter.exit() && !quit_requested; ++frame_index) {
    if (end_index > 0 && frame_index > end_index) break;

    cv::Mat img;
    video.read(img);
    if (img.empty()) break;
    ++total_frames;

    auto timestamp = frame_timestamp(t0, frame_index, metadata.fps);
    std::list<auto_aim::Armor> armors = yolo.detect(img, frame_index);
    armors.remove_if(
      [](const auto_aim::Armor & armor) { return armor.name != auto_aim::ArmorName::outpost; });

    if (!armors.empty()) ++outpost_detected_frames;

    auto targets = tracker.track(armors, timestamp);
    if (!targets.empty()) ++tracked_frames;

    for (const auto & armor : armors) {
      tools::draw_points(img, armor.points, {0, 255, 0});
      tools::draw_text(
        img, fmt::format("{:.2f} {}", armor.confidence, auto_aim::ARMOR_NAMES[armor.name]),
        cv::Point(cvRound(armor.center.x), cvRound(armor.center.y - 12.0f)), {0, 255, 0}, 0.7, 2);
    }

    if (!targets.empty()) {
      const auto & target = targets.front();
      for (const auto & xyza : target.armor_xyza_list()) {
        const auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 255});
      }

      const auto x = target.ekf_x();
      tools::draw_text(
        img,
        fmt::format(
          "tracker:{} last_id:{} w:{:.2f} z:{:.2f} r:{:.3f}", tracker.state(), target.last_id,
          x[7], x[4], x[8]),
        {20, 100}, {0, 255, 255}, 0.7, 2);
    } else {
      tools::draw_text(
        img, fmt::format("tracker:{}", tracker.state()), {20, 100}, {0, 255, 255}, 0.7, 2);
    }

    tools::draw_text(
      img,
      fmt::format(
        "frame:{}/{} fps:{:.2f} size:{}x{} detected:{}/{} tracked:{}/{}", frame_index,
        metadata.total_frames, metadata.fps, metadata.width, metadata.height,
        outpost_detected_frames, total_frames, tracked_frames, total_frames),
      {20, 40}, {255, 255, 255}, 0.8, 2);
    tools::draw_text(
      img, "gimbal quaternion: identity", {20, 70}, {255, 255, 255}, 0.7, 2);

    if (display) {
      cv::Mat display_img = img;
      if (metadata.display_scale != 1.0) {
        cv::resize(img, display_img, {}, metadata.display_scale, metadata.display_scale);
      }
      cv::imshow("outpost_test", display_img);
      const int key = cv::waitKey(metadata.wait_ms);
      if (key == 'q') break;
      if (key == ' ') {
        while (true) {
          const int pause_key = cv::waitKey(30);
          if (pause_key == 'q') {
            quit_requested = true;
            break;
          }
          if (pause_key == ' ') break;
        }
      }
    }
  }

  if (display) cv::destroyAllWindows();

  tools::logger()->info(
    "[outpost_test] outpost detected {}/{} frames, tracked {}/{} frames",
    outpost_detected_frames, total_frames, tracked_frames, total_frames);

  if (total_frames == 0) {
    tools::logger()->error("[outpost_test] no frames were read from {}", video_path);
    return 1;
  }

  return outpost_detected_frames > 0 ? 0 : 1;
}
