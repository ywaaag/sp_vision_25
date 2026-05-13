#include <fmt/core.h>

#include <chrono>
#include <cmath>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double REPORT_INTERVAL_S = 5.0;

const std::string keys =
  "{help h usage ? |                        | output command line help }"
  "{amplitude-deg  | -1                     | override target yaw amplitude in degree }"
  "{frequency-hz   | -1                     | override target yaw frequency in Hz }"
  "{pitch-frequency-hz | -1                 | override target pitch step frequency in Hz }"
  "{pitch-low-deg  | 1000                   | override low target pitch in degree }"
  "{pitch-high-deg | 1000                   | override high target pitch in degree }"
  "{@config-path   | configs/standard3.yaml | yaml config path }";

double read_curve_param(const YAML::Node & curve_yaml, const std::string & key, double fallback)
{
  if (curve_yaml && curve_yaml[key]) return curve_yaml[key].as<double>();
  return fallback;
}

double evaluate_target_yaw(double elapsed_s, double amplitude_rad, double frequency_hz)
{
  auto phase = std::fmod(elapsed_s * frequency_hz, 1.0);
  if (phase < 0) phase += 1.0;
  return -amplitude_rad + 2.0 * amplitude_rad * phase;
}

double evaluate_target_pitch(
  double elapsed_s, double frequency_hz, double low_pitch_rad, double high_pitch_rad)
{
  auto phase = std::fmod(elapsed_s * frequency_hz, 1.0);
  if (phase < 0) phase += 1.0;
  return phase < 0.5 ? low_pitch_rad : high_pitch_rad;
}

auto build_reference(
  double elapsed_s, double yaw_amplitude_rad, double yaw_frequency_hz, double pitch_frequency_hz,
  double pitch_low_rad, double pitch_high_rad)
{
  auto yaw0 = evaluate_target_yaw(elapsed_s, yaw_amplitude_rad, yaw_frequency_hz);
  auto traj = auto_aim::Trajectory::Zero();

  for (int i = 0; i < auto_aim::HORIZON; ++i) {
    auto relative_t = (i - auto_aim::HALF_HORIZON) * auto_aim::DT;
    auto sample_t = elapsed_s + relative_t;
    auto yaw = evaluate_target_yaw(sample_t, yaw_amplitude_rad, yaw_frequency_hz);
    auto yaw_prev = evaluate_target_yaw(sample_t - auto_aim::DT, yaw_amplitude_rad, yaw_frequency_hz);
    auto yaw_next = evaluate_target_yaw(sample_t + auto_aim::DT, yaw_amplitude_rad, yaw_frequency_hz);
    auto yaw_vel = tools::limit_rad(yaw_next - yaw_prev) / (2.0 * auto_aim::DT);
    auto pitch = evaluate_target_pitch(sample_t, pitch_frequency_hz, pitch_low_rad, pitch_high_rad);

    traj.col(i) << tools::limit_rad(yaw - yaw0), yaw_vel, pitch, 0.0;
  }

  return std::pair<double, auto_aim::Trajectory>{yaw0, traj};
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  auto yaml = tools::load(config_path);
  auto curve_yaml = yaml["curve_generator"];

  auto amplitude_deg = read_curve_param(curve_yaml, "amplitude_deg", 10.0);
  auto frequency_hz = read_curve_param(curve_yaml, "frequency_hz", 0.2);
  auto pitch_frequency_hz = read_curve_param(curve_yaml, "pitch_frequency_hz", 0.1);
  auto pitch_low_deg = read_curve_param(curve_yaml, "pitch_low_deg", -2.0);
  auto pitch_high_deg = read_curve_param(curve_yaml, "pitch_high_deg", 2.0);

  const auto cli_amplitude_deg = cli.get<double>("amplitude-deg");
  const auto cli_frequency_hz = cli.get<double>("frequency-hz");
  const auto cli_pitch_frequency_hz = cli.get<double>("pitch-frequency-hz");
  const auto cli_pitch_low_deg = cli.get<double>("pitch-low-deg");
  const auto cli_pitch_high_deg = cli.get<double>("pitch-high-deg");
  if (cli_amplitude_deg > 0) amplitude_deg = cli_amplitude_deg;
  if (cli_frequency_hz > 0) frequency_hz = cli_frequency_hz;
  if (cli_pitch_frequency_hz > 0) pitch_frequency_hz = cli_pitch_frequency_hz;
  if (cli_pitch_low_deg != 1000) pitch_low_deg = cli_pitch_low_deg;
  if (cli_pitch_high_deg != 1000) pitch_high_deg = cli_pitch_high_deg;

  if (amplitude_deg <= 0 || amplitude_deg >= 180.0) {
    tools::logger()->error("curve_generator.amplitude_deg must be in (0, 180), got {}", amplitude_deg);
    return 1;
  }
  if (frequency_hz <= 0) {
    tools::logger()->error("curve_generator.frequency_hz must be > 0, got {}", frequency_hz);
    return 1;
  }
  if (pitch_frequency_hz <= 0) {
    tools::logger()->error(
      "curve_generator.pitch_frequency_hz must be > 0, got {}", pitch_frequency_hz);
    return 1;
  }
  if (pitch_low_deg <= -89.0 || pitch_low_deg >= 89.0) {
    tools::logger()->error(
      "curve_generator.pitch_low_deg must be in (-89, 89), got {}", pitch_low_deg);
    return 1;
  }
  if (pitch_high_deg <= -89.0 || pitch_high_deg >= 89.0) {
    tools::logger()->error(
      "curve_generator.pitch_high_deg must be in (-89, 89), got {}", pitch_high_deg);
    return 1;
  }

  tools::Exiter exiter;
  auto plotter = tools::Plotter::from_config(config_path);

  io::Gimbal gimbal(config_path);
  auto_aim::Planner planner(config_path);

  const auto amplitude_rad = amplitude_deg * DEG_TO_RAD;
  const auto pitch_low_rad = pitch_low_deg * DEG_TO_RAD;
  const auto pitch_high_rad = pitch_high_deg * DEG_TO_RAD;
  const auto start_time = std::chrono::steady_clock::now();
  auto last_report_time = start_time;
  io::GimbalMode last_mode = io::GimbalMode::IDLE;

  std::size_t fire_count = 0;
  std::size_t sample_count = 0;
  auto report_fire_ratio = [&](const std::chrono::steady_clock::time_point & now) {
    const auto report_elapsed_s = tools::delta_time(now, last_report_time);
    if (report_elapsed_s < REPORT_INTERVAL_S) return;

    if (sample_count == 0) {
      tools::logger()->info(
        "[curve_generator] last {:.1f}s no AUTO_AIM samples, yaw amplitude {:.2f} deg @ {:.2f} Hz, pitch {:.2f}/{:.2f} deg @ {:.2f} Hz",
        REPORT_INTERVAL_S, amplitude_deg, frequency_hz, pitch_low_deg, pitch_high_deg,
        pitch_frequency_hz);
    } else {
      const auto fire_ratio = static_cast<double>(fire_count) / static_cast<double>(sample_count);
      tools::logger()->info(
        "[curve_generator] last {:.1f}s theoretical fire ratio {:.2f}% ({}/{}), yaw amplitude {:.2f} deg @ {:.2f} Hz, pitch {:.2f}/{:.2f} deg @ {:.2f} Hz",
        REPORT_INTERVAL_S, fire_ratio * 100.0, fire_count, sample_count, amplitude_deg,
        frequency_hz, pitch_low_deg, pitch_high_deg, pitch_frequency_hz);
    }

    last_report_time = now;
    fire_count = 0;
    sample_count = 0;
  };

  while (!exiter.exit()) {
    const auto now = std::chrono::steady_clock::now();
    const auto mode = gimbal.mode();
    if (mode != last_mode) {
      tools::logger()->info("curve_generator switch to {}", gimbal.str(mode));
      last_mode = mode;
    }
    report_fire_ratio(now);

    if (mode != io::GimbalMode::AUTO_AIM) {
      gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
      std::this_thread::sleep_for(10ms);
      continue;
    }

    auto elapsed_s = tools::delta_time(now, start_time);
    auto [yaw0, traj] = build_reference(
      elapsed_s, amplitude_rad, frequency_hz, pitch_frequency_hz, pitch_low_rad, pitch_high_rad);
    auto plan = planner.plan_trajectory(traj, yaw0);
    auto gs = gimbal.state();

    gimbal.send(
      plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
      plan.pitch_acc);

    ++sample_count;
    if (plan.fire) ++fire_count;

    nlohmann::json data;
    data["t"] = elapsed_s;
    data["curve_yaw_amplitude_deg"] = amplitude_deg;
    data["curve_yaw_frequency_hz"] = frequency_hz;
    data["curve_pitch_frequency_hz"] = pitch_frequency_hz;
    data["curve_pitch_low_deg"] = pitch_low_deg;
    data["curve_pitch_high_deg"] = pitch_high_deg;

    data["gimbal_yaw"] = gs.yaw;
    data["gimbal_yaw_vel"] = gs.yaw_vel;
    data["gimbal_pitch"] = gs.pitch;
    data["gimbal_pitch_vel"] = gs.pitch_vel;

    data["target_yaw"] = plan.target_yaw;
    data["target_pitch"] = plan.target_pitch;

    data["plan_yaw"] = plan.yaw;
    data["plan_yaw_vel"] = plan.yaw_vel;
    data["plan_yaw_acc"] = plan.yaw_acc;

    data["plan_pitch"] = plan.pitch;
    data["plan_pitch_vel"] = plan.pitch_vel;
    data["plan_pitch_acc"] = plan.pitch_acc;

    data["yaw_error"] = gs.yaw - plan.target_yaw;
    data["pitch_error"] = gs.pitch - plan.target_pitch;
    data["fire"] = plan.fire ? 1 : 0;

    plotter.plot(data);

    std::this_thread::sleep_for(10ms);
  }

  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  return 0;
}
