#include <fmt/format.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <list>
#include <opencv2/opencv.hpp>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

#include "io/gimbal/gimbal.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace
{
struct CandidateResult
{
  int delay_ms = 0;
  int samples = 0;
  double yaw_std_deg = std::numeric_limits<double>::infinity();
  double yaw_mean_deg = 0.0;
};

enum class CameraSide
{
  left,
  right
};

constexpr auto USB_STARTUP_SETTLE = 300ms;
constexpr char USB_LEFT_DEVICE[] = "usb_cam_left";
constexpr char USB_RIGHT_DEVICE[] = "usb_cam_right";

CameraSide parse_camera_side(const std::string & value)
{
  if (value == "left" || value == "usb_left") return CameraSide::left;
  if (value == "right" || value == "usb_right") return CameraSide::right;
  throw std::invalid_argument("camera-side must be one of: left, right");
}

const char * camera_side_name(CameraSide side)
{
  switch (side) {
    case CameraSide::left:
      return "usb_left";
    case CameraSide::right:
      return "usb_right";
    default:
      return "unknown";
  }
}

const char * usb_device_name(CameraSide side)
{
  switch (side) {
    case CameraSide::left:
      return USB_LEFT_DEVICE;
    case CameraSide::right:
      return USB_RIGHT_DEVICE;
    default:
      return "";
  }
}

Eigen::Quaterniond usb_world_q(const Eigen::Quaterniond & q)
{
  // Match the omni debug USB branch: keep yaw/roll, clamp pitch to zero.
  auto usb_ypr = tools::eulers(q, 2, 1, 0);
  usb_ypr[1] = 0.0;
  return Eigen::Quaterniond(tools::rotation_matrix(usb_ypr));
}

std::list<auto_aim::Armor> filter_enemy_armors(
  std::list<auto_aim::Armor> armors, auto_aim::Color enemy_color)
{
  armors.remove_if([enemy_color](const auto_aim::Armor & armor) { return armor.color != enemy_color; });
  return armors;
}

void solve_armors(std::list<auto_aim::Armor> & armors, auto_aim::Solver & solver)
{
  for (auto & armor : armors) solver.solve(armor);
}

std::optional<auto_aim::Armor> select_nearest_armor(const std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return std::nullopt;

  auto nearest = armors.front();
  auto min_distance = nearest.xyz_in_gimbal.norm();
  for (const auto & armor : armors) {
    const auto distance = armor.xyz_in_gimbal.norm();
    if (distance < min_distance) {
      min_distance = distance;
      nearest = armor;
    }
  }

  return nearest;
}

double circular_mean(const std::vector<double> & angles)
{
  if (angles.empty()) return 0.0;
  double sin_sum = 0.0;
  double cos_sum = 0.0;
  for (double angle : angles) {
    sin_sum += std::sin(angle);
    cos_sum += std::cos(angle);
  }
  return std::atan2(sin_sum / angles.size(), cos_sum / angles.size());
}

double circular_stddev(const std::vector<double> & angles)
{
  if (angles.size() < 2) return std::numeric_limits<double>::infinity();
  double sin_sum = 0.0;
  double cos_sum = 0.0;
  for (double angle : angles) {
    sin_sum += std::sin(angle);
    cos_sum += std::cos(angle);
  }
  const double mean_sin = sin_sum / angles.size();
  const double mean_cos = cos_sum / angles.size();
  const double r = std::hypot(mean_sin, mean_cos);
  if (r <= 1e-6 || r > 1.0) return std::numeric_limits<double>::infinity();
  return std::sqrt(-2.0 * std::log(r));
}

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | yaml配置文件路径 }"
  "{camera-side    | left                   | 感知相机方向(left / right) }"
  "{delay-min-ms   | 0                      | delay搜索起点(ms) }"
  "{delay-max-ms   | 12                     | delay搜索终点(ms) }"
  "{delay-step-ms  | 1                      | delay搜索步长(ms) }"
  "{scan-amplitude | 30.0                   | 左右摆动幅度(度) }"
  "{scan-period    | 2.0                    | 往返周期(秒) }"
  "{scan-duration  | 6.0                    | 每个delay采样时长(秒) }"
  "{settle-time    | 1.0                    | 每个delay开始前稳定时长(秒) }"
  "{min-samples    | 30                     | 接受结果的最小样本数 }"
  "{display        | true                   | 是否显示识别画面 }";
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  const auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  CameraSide camera_side;
  try {
    camera_side = parse_camera_side(cli.get<std::string>("camera-side"));
  } catch (const std::exception & e) {
    tools::logger()->error("[omni_perception_delay_tuner] {}", e.what());
    return 1;
  }

  const int delay_min_ms = cli.get<int>("delay-min-ms");
  const int delay_max_ms = cli.get<int>("delay-max-ms");
  const int delay_step_ms = std::max(1, cli.get<int>("delay-step-ms"));
  const double scan_amplitude_deg = cli.get<double>("scan-amplitude");
  const double scan_period_s = std::max(0.5, cli.get<double>("scan-period"));
  const double scan_duration_s = std::max(1.0, cli.get<double>("scan-duration"));
  const double settle_time_s = std::max(0.0, cli.get<double>("settle-time"));
  const int min_samples = std::max(1, cli.get<int>("min-samples"));
  const bool display = cli.get<bool>("display");

  auto yaml = tools::load(config_path);
  const auto enemy_color =
    tools::read<std::string>(yaml, "enemy_color") == "red" ? auto_aim::Color::red : auto_aim::Color::blue;

  tools::Exiter exiter;
  io::Gimbal gimbal(config_path);
  io::USBCamera usb_camera(usb_device_name(camera_side), config_path);
  std::this_thread::sleep_for(USB_STARTUP_SETTLE);

  auto_aim::Detector detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Planner planner(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  usb_camera.read(img, t);

  const auto initial_state = gimbal.state();
  const double center_yaw = initial_state.yaw;
  const double center_pitch = initial_state.pitch;
  const double amplitude_rad = scan_amplitude_deg / 57.3;
  const double omega = 2.0 * CV_PI / scan_period_s;

  tools::logger()->info(
    "[omni_perception_delay_tuner] side={} center_yaw={:.2f}deg center_pitch={:.2f}deg amplitude={:.2f}deg",
    camera_side_name(camera_side), center_yaw * 57.3, center_pitch * 57.3, scan_amplitude_deg);

  std::vector<CandidateResult> results;

  for (int delay_ms = delay_min_ms; delay_ms <= delay_max_ms; delay_ms += delay_step_ms) {
    if (exiter.exit()) break;

    tools::logger()->info("[omni_perception_delay_tuner] testing delay {} ms", delay_ms);
    auto_aim::Tracker tracker(config_path, solver);
    std::vector<double> target_yaws;

    const auto phase_begin = std::chrono::steady_clock::now();
    while (!exiter.exit()) {
      usb_camera.read(img, t);

      const auto now = std::chrono::steady_clock::now();
      const double elapsed = tools::delta_time(now, phase_begin);
      if (elapsed > settle_time_s + scan_duration_s) break;

      const double active_t = std::max(0.0, elapsed - settle_time_s);
      const double desired_yaw =
        center_yaw + (elapsed < settle_time_s ? 0.0 : amplitude_rad * std::sin(omega * active_t));
      const double desired_yaw_vel =
        elapsed < settle_time_s ? 0.0 : amplitude_rad * omega * std::cos(omega * active_t);
      const double desired_yaw_acc =
        elapsed < settle_time_s ? 0.0 : -amplitude_rad * omega * omega * std::sin(omega * active_t);

      gimbal.send(true, false, desired_yaw, desired_yaw_vel, desired_yaw_acc, center_pitch, 0.0f, 0.0f);

      const auto q = usb_world_q(gimbal.q(t - std::chrono::milliseconds(delay_ms)));
      const auto gs = gimbal.state();
      solver.set_R_gimbal2world(q);

      auto armors = filter_enemy_armors(detector.detect(img), enemy_color);
      solve_armors(armors, solver);

      std::list<auto_aim::Armor> tracked_armors;
      auto nearest_armor = select_nearest_armor(armors);
      if (nearest_armor.has_value()) tracked_armors.push_back(*nearest_armor);

      auto targets = tracker.track(tracked_armors, t, false);
      if (!targets.empty()) {
        auto plan = planner.plan(targets.front(), gs.bullet_speed);
        if (elapsed >= settle_time_s && plan.control) {
          target_yaws.push_back(plan.target_yaw);
        }

        if (display) {
          auto target = targets.front();
          for (const auto & xyza : target.armor_xyza_list()) {
            auto image_points =
              solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
            tools::draw_points(img, image_points, {0, 255, 0});
          }
        }
      }

      if (display) {
        for (const auto & armor : armors) tools::draw_points(img, armor.points, {0, 255, 0});
        if (nearest_armor.has_value()) tools::draw_points(img, nearest_armor->points, {0, 0, 255});

        tools::draw_text(
          img,
          fmt::format(
            "{} delay={}ms samples={} armors={} yaw={:.2f}",
            camera_side_name(camera_side), delay_ms, target_yaws.size(), armors.size(),
            desired_yaw * 57.3),
          {20, 40}, {0, 255, 255}, 0.8, 2);
        cv::resize(img, img, {}, 0.5, 0.5);
        cv::imshow("omni_perception_delay_tuner", img);
        const auto key = cv::waitKey(1);
        if (key == 'q') {
          gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
          return 0;
        }
      }
    }

    CandidateResult result;
    result.delay_ms = delay_ms;
    result.samples = static_cast<int>(target_yaws.size());
    result.yaw_mean_deg = circular_mean(target_yaws) * 57.3;
    result.yaw_std_deg = circular_stddev(target_yaws) * 57.3;
    results.emplace_back(result);

    if (result.samples >= min_samples) {
      tools::logger()->info(
        "[omni_perception_delay_tuner] side={} delay={}ms samples={} target_yaw_std={:.4f}deg target_yaw_mean={:.4f}deg",
        camera_side_name(camera_side), result.delay_ms, result.samples, result.yaw_std_deg,
        result.yaw_mean_deg);
    } else {
      tools::logger()->warn(
        "[omni_perception_delay_tuner] side={} delay={}ms valid samples too few: {}",
        camera_side_name(camera_side), result.delay_ms, result.samples);
    }

    gimbal.send(true, false, center_yaw, 0.0f, 0.0f, center_pitch, 0.0f, 0.0f);
    std::this_thread::sleep_for(500ms);
  }

  gimbal.send(true, false, center_yaw, 0.0f, 0.0f, center_pitch, 0.0f, 0.0f);

  const auto best_it = std::min_element(
    results.begin(), results.end(), [min_samples](const CandidateResult & a, const CandidateResult & b) {
      const bool a_valid = a.samples >= min_samples && std::isfinite(a.yaw_std_deg);
      const bool b_valid = b.samples >= min_samples && std::isfinite(b.yaw_std_deg);
      if (a_valid != b_valid) return a_valid > b_valid;
      return a.yaw_std_deg < b.yaw_std_deg;
    });

  if (best_it == results.end() || best_it->samples < min_samples || !std::isfinite(best_it->yaw_std_deg)) {
    tools::logger()->error("[omni_perception_delay_tuner] no valid candidate found");
    gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
    return 1;
  }

  tools::logger()->info("========== omni_perception_delay_tuner summary ==========");
  for (const auto & result : results) {
    tools::logger()->info(
      "delay={}ms samples={} target_yaw_std={:.4f}deg target_yaw_mean={:.4f}deg",
      result.delay_ms, result.samples, result.yaw_std_deg, result.yaw_mean_deg);
  }
  tools::logger()->info(
    "[omni_perception_delay_tuner] side={} BEST delay = {} ms (target_yaw_std = {:.4f} deg, samples = {})",
    camera_side_name(camera_side), best_it->delay_ms, best_it->yaw_std_deg, best_it->samples);

  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  if (display) cv::destroyAllWindows();
  return 0;
}
