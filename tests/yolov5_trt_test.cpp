#include <fmt/core.h>

#include <exception>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/yolos/yolov5_trt.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | configs/demo.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                 | 视频起始帧下标    }"
  "{end-index e    | 0                 | 视频结束帧下标    }"
  "{repeat r       | 0                 | 对单帧重复推理次数，0表示正常逐帧跑}"
  "{@input-path    | assets/demo/demo  | avi文件路径去掉后缀}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");
  auto repeat = cli.get<int>("repeat");

  auto video_path = fmt::format("{}.avi", input_path);
  cv::VideoCapture video(video_path);
  if (!video.isOpened()) {
    tools::logger()->error("Failed to open video: {}", video_path);
    return 1;
  }

  try {
    auto_aim::YOLOV5TRT yolo(config_path, false);
    cv::Mat img;

    int processed_frames = 0;
    double total_preprocess_ms = 0.0;
    double total_input_pack_ms = 0.0;
    double total_h2d_ms = 0.0;
    double total_gpu_compute_ms = 0.0;
    double total_d2h_ms = 0.0;
    double total_infer_ms = 0.0;
    double total_postprocess_ms = 0.0;
    double total_total_ms = 0.0;

    video.set(cv::CAP_PROP_POS_FRAMES, start_index);

    if (repeat > 0) {
      cv::Mat bench_img;
      for (int frame_count = 0; frame_count <= start_index; ++frame_count) {
        video.read(bench_img);
        if (bench_img.empty()) {
          tools::logger()->error("Failed to read frame {} from {}", start_index, video_path);
          return 1;
        }
      }

      for (int iter = 0; iter < repeat; ++iter) {
        try {
          const auto armors = yolo.detect(bench_img, start_index);
          const auto & profile = yolo.last_profile();
          if (!profile.valid) {
            tools::logger()->error("[repeat {}] Missing TensorRT timing profile", iter);
            return 2;
          }

          tools::logger()->info(
            "[repeat {}] preprocess: {:.2f}ms, pack: {:.2f}ms, h2d: {:.2f}ms, compute: {:.2f}ms, d2h: {:.2f}ms, infer: {:.2f}ms, postprocess: {:.2f}ms, total: {:.2f}ms, armors: {}",
            iter, profile.preprocess_ms, profile.input_pack_ms, profile.h2d_ms,
            profile.gpu_compute_ms, profile.d2h_ms, profile.infer_ms, profile.postprocess_ms,
            profile.total_ms, armors.size());

          total_preprocess_ms += profile.preprocess_ms;
          total_input_pack_ms += profile.input_pack_ms;
          total_h2d_ms += profile.h2d_ms;
          total_gpu_compute_ms += profile.gpu_compute_ms;
          total_d2h_ms += profile.d2h_ms;
          total_infer_ms += profile.infer_ms;
          total_postprocess_ms += profile.postprocess_ms;
          total_total_ms += profile.total_ms;
          processed_frames++;
        } catch (const std::exception & e) {
          tools::logger()->error("[repeat {}] YOLOV5TRT inference failed: {}", iter, e.what());
          return 2;
        }
      }

      tools::logger()->info(
        "Repeated frame {} for {} runs, avg preprocess: {:.2f}ms, avg pack: {:.2f}ms, avg h2d: {:.2f}ms, avg compute: {:.2f}ms, avg d2h: {:.2f}ms, avg infer: {:.2f}ms, avg postprocess: {:.2f}ms, avg total: {:.2f}ms",
        start_index, processed_frames, total_preprocess_ms / processed_frames,
        total_input_pack_ms / processed_frames, total_h2d_ms / processed_frames,
        total_gpu_compute_ms / processed_frames, total_d2h_ms / processed_frames,
        total_infer_ms / processed_frames, total_postprocess_ms / processed_frames,
        total_total_ms / processed_frames);
      return 0;
    }

    for (int frame_count = start_index; ; ++frame_count) {
      if (end_index > 0 && frame_count > end_index) break;

      video.read(img);
      if (img.empty()) break;

      try {
        const auto armors = yolo.detect(img, frame_count);
        const auto & profile = yolo.last_profile();
        if (!profile.valid) {
          tools::logger()->error("[{}] Missing TensorRT timing profile", frame_count);
          return 2;
        }

        tools::logger()->info(
          "[{}] preprocess: {:.2f}ms, pack: {:.2f}ms, h2d: {:.2f}ms, compute: {:.2f}ms, d2h: {:.2f}ms, infer: {:.2f}ms, postprocess: {:.2f}ms, total: {:.2f}ms, armors: {}",
          frame_count, profile.preprocess_ms, profile.input_pack_ms, profile.h2d_ms,
          profile.gpu_compute_ms, profile.d2h_ms, profile.infer_ms, profile.postprocess_ms,
          profile.total_ms, armors.size());

        total_preprocess_ms += profile.preprocess_ms;
        total_input_pack_ms += profile.input_pack_ms;
        total_h2d_ms += profile.h2d_ms;
        total_gpu_compute_ms += profile.gpu_compute_ms;
        total_d2h_ms += profile.d2h_ms;
        total_infer_ms += profile.infer_ms;
        total_postprocess_ms += profile.postprocess_ms;
        total_total_ms += profile.total_ms;
        processed_frames++;
      } catch (const std::exception & e) {
        tools::logger()->error("[{}] YOLOV5TRT inference failed: {}", frame_count, e.what());
        return 2;
      }
    }

    if (processed_frames == 0) {
      tools::logger()->warn("No frames were processed from {}", video_path);
      return 0;
    }

    tools::logger()->info(
      "Processed {} frames, avg preprocess: {:.2f}ms, avg pack: {:.2f}ms, avg h2d: {:.2f}ms, avg compute: {:.2f}ms, avg d2h: {:.2f}ms, avg infer: {:.2f}ms, avg postprocess: {:.2f}ms, avg total: {:.2f}ms",
      processed_frames, total_preprocess_ms / processed_frames,
      total_input_pack_ms / processed_frames, total_h2d_ms / processed_frames,
      total_gpu_compute_ms / processed_frames, total_d2h_ms / processed_frames,
      total_infer_ms / processed_frames, total_postprocess_ms / processed_frames,
      total_total_ms / processed_frames);
  } catch (const std::exception & e) {
    tools::logger()->error("Failed to initialize YOLOV5TRT: {}", e.what());
    return 1;
  }

  return 0;
}
