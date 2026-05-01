#include <fmt/core.h>

#include <Eigen/Geometry>
#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "src/auto_aim_debug_dashboard.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/dashboard_cli.hpp"
#include "tools/dashboard_config.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{dashboard      |                        | 启用 MQTT Dashboard}"
  "{robot-id       | myrobot                | MQTT Dashboard robot id}"
  "{mqtt-host      | tcp://127.0.0.1:1883   | MQTT broker URI}"
  "{video-source   | assets/demo/demo.avi   | 无硬件测试视频路径}"
  "{video-loop     |                        | 视频结束后循环读取}"
  "{max-frames     | 0                      | 最大处理帧数，0 表示直到视频结束}"
  "{bullet-speed   | 22.0                   | 规划使用的固定弹速 m/s}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径}";

std::vector<std::string> normalize_cli_args(int argc, char * argv[])
{
  const std::set<std::string> value_options{
    "--robot-id", "--mqtt-host", "--video-source", "--max-frames", "--bullet-speed"};

  std::vector<std::string> normalized;
  normalized.reserve(argc);
  for (int i = 0; i < argc; ++i) {
    const std::string arg = argv[i];
    if (value_options.count(arg) > 0 && i + 1 < argc) {
      normalized.push_back(arg + "=" + argv[++i]);
    } else {
      normalized.push_back(arg);
    }
  }
  return normalized;
}

std::string sibling_pose_path(const std::string & video_path)
{
  auto pose_path = video_path;
  const auto slash = pose_path.find_last_of("/\\");
  const auto dot = pose_path.find_last_of('.');
  if (dot != std::string::npos && (slash == std::string::npos || dot > slash)) {
    pose_path.replace(dot, std::string::npos, ".txt");
  } else {
    pose_path += ".txt";
  }
  return pose_path;
}

class VideoPoseSource
{
public:
  VideoPoseSource(const std::string & video_path, bool loop)
  : video_path_(video_path), pose_path_(sibling_pose_path(video_path)), loop_(loop)
  {
    video_.open(video_path_);
    if (!video_.isOpened()) {
      return;
    }

    fps_ = video_.get(cv::CAP_PROP_FPS);
    if (fps_ <= 1e-6) {
      fps_ = 30.0;
      tools::logger()->warn("Video FPS unavailable, fallback to {:.1f} FPS", fps_);
    }

    pose_.open(pose_path_);
    if (!pose_.is_open()) {
      tools::logger()->warn("Pose file {} not found, using identity gimbal pose", pose_path_);
    }
  }

  bool opened() const { return video_.isOpened(); }

  bool read(
    cv::Mat & img, std::chrono::steady_clock::time_point & timestamp, Eigen::Quaterniond & q)
  {
    while (true) {
      video_.read(img);
      if (!img.empty()) {
        break;
      }

      if (!loop_) {
        return false;
      }

      video_.set(cv::CAP_PROP_POS_FRAMES, 0);
      reset_pose();
    }

    q = next_pose();
    timestamp =
      t0_ + std::chrono::microseconds(static_cast<int64_t>(global_frame_ * 1e6 / fps_));
    ++global_frame_;
    return true;
  }

private:
  void reset_pose()
  {
    if (!pose_.is_open()) {
      return;
    }

    pose_.clear();
    pose_.seekg(0);
  }

  Eigen::Quaterniond next_pose()
  {
    double pose_time = 0.0;
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    if (pose_.is_open()) {
      if (!(pose_ >> pose_time >> w >> x >> y >> z)) {
        if (loop_) {
          reset_pose();
          if (!(pose_ >> pose_time >> w >> x >> y >> z)) {
            return Eigen::Quaterniond::Identity();
          }
        } else {
          if (!pose_exhausted_warned_) {
            tools::logger()->warn(
              "Pose file {} ended before video, using identity gimbal pose", pose_path_);
            pose_exhausted_warned_ = true;
          }
          return Eigen::Quaterniond::Identity();
        }
      }
    }
    (void)pose_time;

    Eigen::Quaterniond q(w, x, y, z);
    if (q.norm() < 1e-9) {
      return Eigen::Quaterniond::Identity();
    }
    return q.normalized();
  }

  std::string video_path_;
  std::string pose_path_;
  bool loop_;
  bool pose_exhausted_warned_{false};
  double fps_{30.0};
  int64_t global_frame_{0};
  std::chrono::steady_clock::time_point t0_{std::chrono::steady_clock::now()};
  cv::VideoCapture video_;
  std::ifstream pose_;
};

void add_target_telemetry(nlohmann::json & data, const auto_aim::Target & target)
{
  const auto x = target.ekf_x();
  data["target_x"] = x[0];
  data["target_vx"] = x[1];
  data["target_y"] = x[2];
  data["target_vy"] = x[3];
  data["target_z"] = x[4];
  data["target_vz"] = x[5];
  data["target_a"] = x[6] * 57.3;
  data["w"] = x[7];
  data["target_r"] = x[8];
  data["target_l"] = x[9];
  data["target_h"] = x[10];
  data["last_id"] = target.last_id;
}

int main(int argc, char * argv[])
{
  auto normalized_args = normalize_cli_args(argc, argv);
  auto normalized_argv = tools::dashboard::cli::make_cli_argv(normalized_args);
  cv::CommandLineParser cli(
    static_cast<int>(normalized_argv.size()), normalized_argv.data(), keys);

  const auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  const auto video_path = cli.get<std::string>("video-source");
  const auto max_frames = cli.get<int>("max-frames");
  const auto bullet_speed = cli.get<double>("bullet-speed");
  const auto video_loop = cli.has("video-loop");
  if (!cli.check()) {
    cli.printErrors();
    return 1;
  }

  VideoPoseSource source(video_path, video_loop);
  if (!source.opened()) {
    tools::logger()->error("Failed to open hardwareless video source: {}", video_path);
    return 1;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  const auto dashboard_config = tools::dashboard::load_dashboard_config(
    config_path,
    tools::dashboard::cli::make_dashboard_overrides(normalized_args, cli.has("dashboard")));

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);
  AutoAimDebugDashboard dashboard(dashboard_config, config_path, planner);

  tools::logger()->info(
    "Starting hardwareless dashboard smoke with video={}, config={}, max_frames={}", video_path,
    config_path, max_frames);

  cv::Mat img;
  auto timestamp = std::chrono::steady_clock::now();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  auto run_start = std::chrono::steady_clock::now();

  for (int frame_count = 0; !exiter.exit(); ++frame_count) {
    if (max_frames > 0 && frame_count >= max_frames) {
      break;
    }

    dashboard.handle_commands();

    if (!source.read(img, timestamp, q)) {
      break;
    }

    solver.set_R_gimbal2world(q);

    const auto yolo_start = std::chrono::steady_clock::now();
    auto armors = yolo.detect(img, frame_count);

    const auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, timestamp);
    std::optional<auto_aim::Target> target;
    if (!targets.empty()) {
      target = targets.front();
    }

    const auto planner_start = std::chrono::steady_clock::now();
    const auto plan = planner.plan(target, bullet_speed);
    const auto finish = std::chrono::steady_clock::now();

    nlohmann::json data;
    data["t"] = tools::delta_time(finish, run_start);
    data["frame"] = frame_count;
    data["video_source"] = video_path;
    data["armor_num"] = armors.size();
    data["target_found"] = target.has_value();
    data["bullet_speed"] = bullet_speed;
    data["yolo_ms"] = tools::delta_time(tracker_start, yolo_start) * 1e3;
    data["tracker_ms"] = tools::delta_time(planner_start, tracker_start) * 1e3;
    data["planner_ms"] = tools::delta_time(finish, planner_start) * 1e3;

    if (!armors.empty()) {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_z"] = armor.xyz_in_world[2];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
      data["armor_center_x"] = armor.center_norm.x;
      data["armor_center_y"] = armor.center_norm.y;
    }

    data["plan_control"] = plan.control ? 1 : 0;
    data["fire"] = plan.fire ? 1 : 0;
    data["target_yaw"] = plan.target_yaw;
    data["target_pitch"] = plan.target_pitch;
    data["plan_yaw"] = plan.yaw;
    data["plan_yaw_vel"] = plan.yaw_vel;
    data["plan_yaw_acc"] = plan.yaw_acc;
    data["plan_pitch"] = plan.pitch;
    data["plan_pitch_vel"] = plan.pitch_vel;
    data["plan_pitch_acc"] = plan.pitch_acc;

    if (target.has_value()) {
      add_target_telemetry(data, *target);
    } else {
      data["w"] = 0.0;
    }

    plotter.plot(data);
    dashboard.push_data(data);
    std::this_thread::sleep_for(10ms);
  }

  dashboard.stop();
  tools::logger()->info("Hardwareless dashboard smoke stopped");
  return 0;
}
