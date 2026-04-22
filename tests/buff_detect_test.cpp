#include <fmt/core.h>

#include <opencv2/opencv.hpp>

#include "tasks/auto_buff/buff_detector.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ? |                         | 输出命令行参数说明 }"
  "{config-path c  | configs/standard3.yaml  | yaml配置文件的路径}"
  "{display d      | true                    | 是否显示检测窗口  }"
  "{start-index s  | 0                       | 视频起始帧下标    }"
  "{end-index e    | 0                       | 视频结束帧下标    }"
  "{@video-path    | assets/demo.avi         | 待检测avi路径     }";

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
  auto_buff::Buff_Detector detector(config_path);

  cv::VideoCapture video(video_path);
  if (!video.isOpened()) {
    tools::logger()->error("[buff_detect_test] failed to open video: {}", video_path);
    return 1;
  }

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);

  int detected_frames = 0;
  int total_frames = 0;

  for (int frame_count = start_index; !exiter.exit(); ++frame_count) {
    if (end_index > 0 && frame_count > end_index) break;

    cv::Mat img;
    video.read(img);
    if (img.empty()) break;

    ++total_frames;
    auto power_rune = detector.detect(img);
    if (power_rune.has_value()) {
      ++detected_frames;

      auto & rune = power_rune.value();
      auto & target = rune.target();
      for (int i = 0; i < 4; ++i) tools::draw_point(img, target.points[i], {0, 255, 0}, 3);
      tools::draw_point(img, target.center, {0, 0, 255}, 4);
      tools::draw_point(img, rune.r_center, {255, 255, 0}, 4);
    }

    tools::draw_text(
      img,
      fmt::format(
        "frame:{} detect:{}/{}", frame_count, detected_frames, total_frames),
      {20, 70}, {0, 255, 0});

    if (display) {
      cv::imshow("buff_detect_test", img);
      const int key = cv::waitKey(30);
      if (key == 'q') break;
    }
  }

  if (display) cv::destroyAllWindows();

  tools::logger()->info(
    "[buff_detect_test] detected {}/{} frames from {}", detected_frames, total_frames, video_path);

  if (total_frames == 0) {
    tools::logger()->error("[buff_detect_test] no frames were read from {}", video_path);
    return 1;
  }

  return detected_frames > 0 ? 0 : 1;
}
