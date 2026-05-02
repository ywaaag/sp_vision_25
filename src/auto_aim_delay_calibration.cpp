#include <fmt/core.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <list>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>
#include <vector>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

namespace
{
using Clock = std::chrono::steady_clock;

const std::string keys =
  "{help h usage ? |                        | output command line help }"
  "{@config-path   | configs/standard3.yaml | yaml config path }"
  "{freq-hz        | 0.8                    | yaw sine frequency in Hz }"
  "{amp-deg        | 8.0                    | yaw sine amplitude in degrees }"
  "{delay-min-ms   | -4.0                   | minimum IMU delay in milliseconds }"
  "{delay-max-ms   | 12.0                   | maximum IMU delay in milliseconds }"
  "{delay-step-ms  | 1.0                    | IMU delay step in milliseconds }"
  "{sample-sec     | 5.0                    | sampling time per delay in seconds }"
  "{warmup-sec     | 2.0                    | warmup time before sweep in seconds }"
  "{min-samples    | 30                     | minimum valid samples per delay }"
  "{show           | 1                      | show calibration window, 1 or 0 }";

Clock::duration to_duration_ms(double delay_ms)
{
  return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double, std::milli>(delay_ms));
}

double rad2deg(double rad) { return rad * 180.0 / CV_PI; }

struct PoseAccumulator
{
  std::size_t samples = 0;
  Eigen::Vector3d sum_xyz = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_xyz_sq = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_ypr = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_ypr_sq = Eigen::Vector3d::Zero();
  double sum_distance = 0.0;

  void add(const auto_aim::Armor & armor)
  {
    samples++;
    sum_xyz += armor.xyz_in_world;
    sum_xyz_sq += armor.xyz_in_world.cwiseProduct(armor.xyz_in_world);
    sum_ypr += armor.ypr_in_world;
    sum_ypr_sq += armor.ypr_in_world.cwiseProduct(armor.ypr_in_world);
    sum_distance += armor.xyz_in_world.norm();
  }
};

struct CandidateResult
{
  double delay_ms = 0.0;
  std::size_t samples = 0;
  bool valid = false;
  Eigen::Vector3d mean_xyz = Eigen::Vector3d::Zero();
  Eigen::Vector3d std_xyz = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_ypr = Eigen::Vector3d::Zero();
  Eigen::Vector3d std_ypr = Eigen::Vector3d::Zero();
  double mean_distance = 0.0;
  double score = std::numeric_limits<double>::infinity();
};

CandidateResult finalize_result(double delay_ms, const PoseAccumulator & accumulator, int min_samples)
{
  CandidateResult result;
  result.delay_ms = delay_ms;
  result.samples = accumulator.samples;
  result.valid = accumulator.samples >= static_cast<std::size_t>(min_samples);

  if (!result.valid) return result;

  const double count = static_cast<double>(accumulator.samples);
  result.mean_xyz = accumulator.sum_xyz / count;
  result.mean_ypr = accumulator.sum_ypr / count;

  const Eigen::Vector3d var_xyz =
    accumulator.sum_xyz_sq / count - result.mean_xyz.cwiseProduct(result.mean_xyz);
  const Eigen::Vector3d var_ypr =
    accumulator.sum_ypr_sq / count - result.mean_ypr.cwiseProduct(result.mean_ypr);

  result.std_xyz = var_xyz.cwiseMax(0.0).cwiseSqrt();
  result.std_ypr = var_ypr.cwiseMax(0.0).cwiseSqrt();
  result.mean_distance = accumulator.sum_distance / count;
  result.score = result.std_xyz.norm();
  return result;
}

struct TargetSignature
{
  auto_aim::ArmorName name;
  auto_aim::ArmorType type;
  auto_aim::Color color;
};

bool same_target(const auto_aim::Armor & armor, const TargetSignature & signature)
{
  return armor.name == signature.name && armor.type == signature.type && armor.color == signature.color;
}

std::optional<auto_aim::Armor> select_armor(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return std::nullopt;

  auto it = std::max_element(
    armors.begin(), armors.end(), [](const auto_aim::Armor & lhs, const auto_aim::Armor & rhs) {
      return lhs.confidence < rhs.confidence;
    });

  if (it == armors.end()) return std::nullopt;
  return *it;
}

void log_result(const CandidateResult & result)
{
  const auto std_ypr_deg = result.std_ypr * (180.0 / CV_PI);
  tools::logger()->info(
    "delay={:.2f} ms score={:.5f} m samples={} valid={} std_xyz=[{:.4f}, {:.4f}, {:.4f}] m std_ypr=[{:.2f}, {:.2f}, {:.2f}] deg",
    result.delay_ms, result.score, result.samples, result.valid ? "yes" : "no", result.std_xyz[0],
    result.std_xyz[1], result.std_xyz[2], std_ypr_deg[0], std_ypr_deg[1], std_ypr_deg[2]);
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  const auto config_path = cli.get<std::string>("@config-path");
  const auto freq_hz = cli.get<double>("freq-hz");
  const auto amp_deg = cli.get<double>("amp-deg");
  auto delay_min_ms = cli.get<double>("delay-min-ms");
  auto delay_max_ms = cli.get<double>("delay-max-ms");
  const auto delay_step_ms = cli.get<double>("delay-step-ms");
  const auto sample_sec = cli.get<double>("sample-sec");
  const auto warmup_sec = cli.get<double>("warmup-sec");
  const auto min_samples = cli.get<int>("min-samples");
  const bool show = cli.get<int>("show") != 0;

  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  if (freq_hz <= 0.0 || amp_deg <= 0.0 || delay_step_ms <= 0.0 || sample_sec <= 0.0 || warmup_sec < 0.0 ||
      min_samples <= 0) {
    tools::logger()->error("Invalid calibration arguments.");
    return 1;
  }

  if (delay_min_ms > delay_max_ms) std::swap(delay_min_ms, delay_max_ms);

  std::vector<double> delay_candidates;
  for (double delay_ms = delay_min_ms; delay_ms <= delay_max_ms + delay_step_ms * 0.5; delay_ms += delay_step_ms) {
    delay_candidates.push_back(delay_ms);
  }

  if (delay_candidates.empty()) {
    tools::logger()->error("No delay candidates generated.");
    return 1;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  const auto initial_state = gimbal.state();
  const double center_yaw = initial_state.yaw;
  const double hold_pitch = initial_state.pitch;
  const double amplitude_rad = amp_deg * CV_PI / 180.0;
  const double omega = 2.0 * CV_PI * freq_hz;

  tools::logger()->info(
    "Start auto calibration: freq={:.3f} Hz amp={:.2f} deg delay=[{:.2f}, {:.2f}] ms step={:.2f} ms sample={:.2f} s warmup={:.2f} s",
    freq_hz, amp_deg, delay_min_ms, delay_max_ms, delay_step_ms, sample_sec, warmup_sec);
  tools::logger()->info("Keep one static armor in the field of view during the whole scan.");

  std::atomic<bool> quit = false;
  const auto motion_start = Clock::now();
  auto control_thread = std::thread([&]() {
    while (!quit) {
      const double elapsed = tools::delta_time(Clock::now(), motion_start);
      const double phase = omega * elapsed;
      const float yaw = static_cast<float>(center_yaw + amplitude_rad * std::sin(phase));
      const float yaw_vel = static_cast<float>(amplitude_rad * omega * std::cos(phase));
      const float yaw_acc = static_cast<float>(-amplitude_rad * omega * omega * std::sin(phase));
      gimbal.send(true, false, yaw, yaw_vel, yaw_acc, static_cast<float>(hold_pitch), 0.0f, 0.0f);
      std::this_thread::sleep_for(10ms);
    }
  });

  std::vector<CandidateResult> results;
  PoseAccumulator accumulator;
  std::optional<TargetSignature> locked_target;
  std::size_t candidate_index = 0;
  bool warmup_done = false;
  auto stage_start = Clock::now();

  cv::Mat img;
  Clock::time_point t;

  while (!exiter.exit() && candidate_index < delay_candidates.size()) {
    camera.read(img, t);

    const auto now = Clock::now();
    if (!warmup_done) {
      warmup_done = tools::delta_time(now, stage_start) >= warmup_sec;
      if (warmup_done) {
        stage_start = now;
        tools::logger()->info("Warmup finished, start delay sweep.");
      }
    }

    const double current_delay_ms = delay_candidates[candidate_index];
    const auto q = gimbal.q(t - to_duration_ms(current_delay_ms));
    solver.set_R_gimbal2world(q);

    auto armors = yolo.detect(img);
    auto selected = select_armor(armors);
    bool accepted = false;

    if (selected.has_value()) {
      auto armor = *selected;
      solver.solve(armor);

      if (!locked_target.has_value()) {
        locked_target = TargetSignature{armor.name, armor.type, armor.color};
      }

      if (same_target(armor, *locked_target)) {
        accepted = true;
        if (warmup_done) accumulator.add(armor);

        nlohmann::json data;
        data["delay_ms"] = current_delay_ms;
        data["samples"] = accumulator.samples;
        data["armor_x"] = armor.xyz_in_world[0];
        data["armor_y"] = armor.xyz_in_world[1];
        data["armor_z"] = armor.xyz_in_world[2];
        data["armor_yaw"] = rad2deg(armor.ypr_in_world[0]);
        data["armor_pitch"] = rad2deg(armor.ypr_in_world[1]);
        data["armor_roll"] = rad2deg(armor.ypr_in_world[2]);
        data["center_x"] = armor.center_norm.x;
        data["center_y"] = armor.center_norm.y;
        plotter.plot(data);

        if (show) {
          tools::draw_points(img, armor.points, {0, 255, 0});
          tools::draw_text(
            img,
            fmt::format(
              "{} {} conf={:.2f}", auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name],
              armor.confidence),
            cv::Point(20, 40), {0, 255, 0}, 0.8, 2);
        }
      }
    }

    if (warmup_done && tools::delta_time(now, stage_start) >= sample_sec) {
      auto result = finalize_result(current_delay_ms, accumulator, min_samples);
      results.push_back(result);
      log_result(result);

      accumulator = PoseAccumulator{};
      stage_start = now;
      candidate_index++;
    }

    if (show) {
      double progress = 0.0;
      if (warmup_done) {
        progress = tools::delta_time(now, stage_start) / sample_sec;
      } else {
        progress = warmup_sec > 0.0 ? tools::delta_time(now, stage_start) / warmup_sec : 1.0;
      }
      progress = std::clamp(progress, 0.0, 1.0);

      tools::draw_text(
        img,
        warmup_done ? fmt::format("delay {:.2f} ms ({}/{})", current_delay_ms, candidate_index + 1,
                                  delay_candidates.size())
                    : "warmup",
        cv::Point(20, 80), {0, 255, 255}, 0.8, 2);
      tools::draw_text(
        img, fmt::format("freq {:.2f} Hz amp {:.2f} deg", freq_hz, amp_deg), cv::Point(20, 120),
        {0, 255, 255}, 0.8, 2);
      tools::draw_text(
        img, fmt::format("accepted {} progress {:>3.0f}%", accepted ? "yes" : "no", progress * 100.0),
        cv::Point(20, 160), {0, 255, 255}, 0.8, 2);

      cv::resize(img, img, {}, 0.5, 0.5);
      cv::imshow("auto_aim_delay_calibration", img);
      if (cv::waitKey(1) == 'q') break;
    }
  }

  quit = true;
  if (control_thread.joinable()) control_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  if (results.empty()) {
    tools::logger()->warn("No calibration result generated.");
    return 0;
  }

  tools::logger()->info("Calibration summary:");
  for (const auto & result : results) log_result(result);

  std::optional<CandidateResult> best_result;
  for (const auto & result : results) {
    if (!result.valid) continue;
    if (!best_result.has_value() || result.score < best_result->score) best_result = result;
  }

  if (!best_result.has_value()) {
    tools::logger()->warn("No valid delay candidate found. Try increasing sample time or keeping only one armor visible.");
    return 0;
  }

  tools::logger()->info(
    "Recommended imu delay = {:.2f} ms, score = {:.5f} m, std_xyz = [{:.4f}, {:.4f}, {:.4f}] m",
    best_result->delay_ms, best_result->score, best_result->std_xyz[0], best_result->std_xyz[1],
    best_result->std_xyz[2]);
  return 0;
}
