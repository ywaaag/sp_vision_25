#include <fmt/core.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <list>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>

#include "combat_rm_interfaces/msg/target.hpp"
#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "io/ros2/ros2.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tasks/auto_buff/yolo11_buff.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr auto GIMBAL_DELAY = std::chrono::milliseconds(7);
constexpr auto USB_STARTUP_SETTLE = std::chrono::milliseconds(300);
constexpr double USB_LEFT_YAW_OFFSET = 2.7;
constexpr double USB_RIGHT_YAW_OFFSET = -2.7;
constexpr double USB_LEFT_TARGET_YAW_TRIM = -0.44;
constexpr double USB_RIGHT_TARGET_YAW_TRIM = 0.2;
constexpr double USB_TARGET_PITCH_TRIM = 0.3;
constexpr char USB_LEFT_DEVICE[] = "video2";
constexpr char USB_RIGHT_DEVICE[] = "video0";
constexpr float BIG_BUFF_R_DISTANCE_KEEP_RATIO = 0.8F;
constexpr float BIG_BUFF_TRACK_MATCH_DISTANCE = 80.0F;
constexpr int BIG_BUFF_MAX_TRACKS = 2;
constexpr int BIG_BUFF_MAX_LOST_FRAMES = 7;

enum class TargetSource
{
  none = 0,
  main = 1,
  usb_left = 2,
  usb_right = 3
};

struct TargetCommand
{
  std::optional<auto_aim::Target> target = std::nullopt;
  TargetSource source = TargetSource::none;
};

struct UsbCandidate
{
  auto_aim::Armor armor;
  std::chrono::steady_clock::time_point timestamp;
  TargetSource source;
  double distance;
};

struct UsbThreadResult
{
  cv::Mat left_img;
  cv::Mat right_img;
  std::list<auto_aim::Armor> left_armors;
  std::list<auto_aim::Armor> right_armors;
  std::optional<UsbCandidate> candidate = std::nullopt;
  TargetCommand target_command;
  bool ready = false;
};

struct BigBuffCandidate
{
  auto_buff::YOLO11_BUFF::Object object;
  auto_buff::PowerRune power_rune;
  cv::Point2f center;
  float r_distance = 0.0F;
};

struct BigBuffTrack
{
  int id = 0;
  auto_buff::BigTarget target;
  std::optional<auto_buff::PowerRune> power_rune = std::nullopt;
  cv::Point2f center;
  float confidence = 0.0F;
  int lost_count = 0;
};

const char * source_name(TargetSource source)
{
  switch (source) {
    case TargetSource::main:
      return "main";
    case TargetSource::usb_left:
      return "usb_left";
    case TargetSource::usb_right:
      return "usb_right";
    default:
      return "none";
  }
}

double usb_yaw_offset(TargetSource source)
{
  switch (source) {
    case TargetSource::usb_left:
      return USB_LEFT_YAW_OFFSET;
    case TargetSource::usb_right:
      return USB_RIGHT_YAW_OFFSET;
    default:
      return 0.0;
  }
}

double usb_target_yaw_trim(TargetSource source)
{
  switch (source) {
    case TargetSource::usb_left:
      return USB_LEFT_TARGET_YAW_TRIM;
    case TargetSource::usb_right:
      return USB_RIGHT_TARGET_YAW_TRIM;
    default:
      return 0.0;
  }
}

bool should_track_armor(auto_aim::ArmorName name, io::GimbalMode mode)
{
  if (mode == io::GimbalMode::AUTO_AIM) return name != auto_aim::ArmorName::outpost;
  if (mode == io::GimbalMode::OUTPOST) return name == auto_aim::ArmorName::outpost;
  return false;
}


std::string armor_number(auto_aim::ArmorName name)
{
  switch (name) {
    case auto_aim::ArmorName::one:
      return "1";
    case auto_aim::ArmorName::two:
      return "2";
    case auto_aim::ArmorName::three:
      return "3";
    case auto_aim::ArmorName::four:
      return "4";
    case auto_aim::ArmorName::five:
      return "5";
    case auto_aim::ArmorName::outpost:
      return "6";
    case auto_aim::ArmorName::sentry:
      return "7";
    case auto_aim::ArmorName::base:
      return "0";
    case auto_aim::ArmorName::not_armor:
      return "";
  }

  return "";
}

cv::Point2f four_point_center(const std::vector<cv::Point2f> & points)
{
  cv::Point2f center{0.0F, 0.0F};
  for (int i = 0; i < 4; ++i) center += points[i];
  return center / 4.0F;
}

std::optional<BigBuffCandidate> make_big_buff_candidate(
  const auto_buff::YOLO11_BUFF::Object & object)
{
  if (object.kpt.size() < 6) return std::nullopt;

  std::vector<cv::Point2f> blade_points(object.kpt.begin(), object.kpt.begin() + 4);
  auto filter_center = four_point_center(object.kpt);
  std::vector<auto_buff::FanBlade> fanblades;
  fanblades.emplace_back(blade_points, object.kpt[4], auto_buff::_light);
  auto_buff::PowerRune power_rune(fanblades, object.kpt[5], std::nullopt);
  if (power_rune.is_unsolve()) return std::nullopt;

  BigBuffCandidate candidate;
  candidate.object = object;
  candidate.power_rune = power_rune;
  candidate.center = object.kpt[4];
  candidate.r_distance = cv::norm(object.kpt[5] - filter_center);
  return candidate;
}

std::vector<BigBuffCandidate> filter_big_buff_candidates(
  const std::vector<auto_buff::YOLO11_BUFF::Object> & objects)
{
  std::vector<BigBuffCandidate> candidates;
  candidates.reserve(objects.size());

  for (const auto & object : objects) {
    auto candidate = make_big_buff_candidate(object);
    if (candidate.has_value()) candidates.emplace_back(candidate.value());
  }
  if (candidates.empty()) return candidates;

  const auto max_r_it = std::max_element(
    candidates.begin(), candidates.end(), [](const BigBuffCandidate & a, const BigBuffCandidate & b) {
      return a.r_distance < b.r_distance;
    });
  const auto min_keep_r = max_r_it->r_distance * BIG_BUFF_R_DISTANCE_KEEP_RATIO;

  candidates.erase(
    std::remove_if(
      candidates.begin(), candidates.end(),
      [min_keep_r](const BigBuffCandidate & candidate) {
        return candidate.r_distance < min_keep_r;
      }),
    candidates.end());

  std::sort(
    candidates.begin(), candidates.end(), [](const BigBuffCandidate & a, const BigBuffCandidate & b) {
      return a.object.prob > b.object.prob;
    });

  if (candidates.size() > BIG_BUFF_MAX_TRACKS) candidates.resize(BIG_BUFF_MAX_TRACKS);
  return candidates;
}

void update_big_buff_tracks(
  const std::vector<BigBuffCandidate> & candidates, std::vector<BigBuffTrack> & tracks,
  std::optional<int> & locked_track_id, int & next_track_id)
{
  std::vector<bool> candidate_matched(candidates.size(), false);
  std::vector<bool> track_matched(tracks.size(), false);

  for (std::size_t candidate_i = 0; candidate_i < candidates.size(); ++candidate_i) {
    const auto & candidate = candidates[candidate_i];
    std::optional<std::size_t> best_track_i;
    auto best_distance = BIG_BUFF_TRACK_MATCH_DISTANCE;

    for (std::size_t track_i = 0; track_i < tracks.size(); ++track_i) {
      if (track_matched[track_i]) continue;

      const auto distance = static_cast<float>(cv::norm(candidate.center - tracks[track_i].center));
      if (distance < best_distance) {
        best_distance = distance;
        best_track_i = track_i;
      }
    }

    if (!best_track_i.has_value()) continue;

    auto & track = tracks[best_track_i.value()];
    track.power_rune = candidate.power_rune;
    track.center = candidate.center;
    track.confidence = candidate.object.prob;
    track.lost_count = 0;
    candidate_matched[candidate_i] = true;
    track_matched[best_track_i.value()] = true;
  }

  for (std::size_t track_i = 0; track_i < tracks.size(); ++track_i) {
    if (!track_matched[track_i]) ++tracks[track_i].lost_count;
  }

  tracks.erase(
    std::remove_if(
      tracks.begin(), tracks.end(),
      [](const BigBuffTrack & track) { return track.lost_count > BIG_BUFF_MAX_LOST_FRAMES; }),
    tracks.end());

  for (std::size_t candidate_i = 0; candidate_i < candidates.size(); ++candidate_i) {
    if (candidate_matched[candidate_i] || tracks.size() >= BIG_BUFF_MAX_TRACKS) continue;

    BigBuffTrack track;
    track.id = next_track_id++;
    track.power_rune = candidates[candidate_i].power_rune;
    track.center = candidates[candidate_i].center;
    track.confidence = candidates[candidate_i].object.prob;
    tracks.emplace_back(std::move(track));
  }

  if (
    locked_track_id.has_value() &&
    std::none_of(tracks.begin(), tracks.end(), [locked_track_id](const BigBuffTrack & track) {
      return track.id == locked_track_id.value();
    }))
  {
    locked_track_id = std::nullopt;
  }

  if (!locked_track_id.has_value()) {
    auto best_track_it = std::max_element(
      tracks.begin(), tracks.end(), [](const BigBuffTrack & a, const BigBuffTrack & b) {
        if ((a.lost_count == 0) != (b.lost_count == 0)) return a.lost_count != 0;
        return a.confidence < b.confidence;
      });
    if (best_track_it != tracks.end() && best_track_it->lost_count == 0) {
      locked_track_id = best_track_it->id;
    }
  }
}

BigBuffTrack * find_big_buff_track(
  std::vector<BigBuffTrack> & tracks, const std::optional<int> & track_id)
{
  if (!track_id.has_value()) return nullptr;

  auto track_it = std::find_if(tracks.begin(), tracks.end(), [track_id](const BigBuffTrack & track) {
    return track.id == track_id.value();
  });
  return track_it == tracks.end() ? nullptr : &(*track_it);
}

combat_rm_interfaces::msg::Target make_target_msg(const TargetCommand & target_command)
{
  combat_rm_interfaces::msg::Target msg;
  msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  msg.header.frame_id = "vision_world";

  if (!target_command.target.has_value()) {
    msg.tracking = false;
    return msg;
  }

  const auto & target = target_command.target.value();
  const auto ekf_x = target.ekf_x();
  msg.tracking = true;
  msg.id = armor_number(target.name);
  msg.position.x = ekf_x[0];
  msg.position.y = ekf_x[2];
  msg.position.z = ekf_x[4];

  return msg;
}

Eigen::Quaterniond usb_world_q(const Eigen::Quaterniond & q)
{
  // The side USB cameras do not follow gimbal pitch; keep yaw/roll and clamp pitch to zero.
  auto usb_ypr = tools::eulers(q, 2, 1, 0);
  usb_ypr[1] = 0.0;
  return Eigen::Quaterniond(tools::rotation_matrix(usb_ypr));
}

std::list<auto_aim::Armor> filter_armors(
  std::list<auto_aim::Armor> armors, auto_aim::Color enemy_color, io::GimbalMode mode)
{
  armors.remove_if([enemy_color, mode](const auto_aim::Armor & armor) {
    return armor.color != enemy_color || !should_track_armor(armor.name, mode);
  });
  return armors;
}

void solve_armors(std::list<auto_aim::Armor> & armors, auto_aim::Solver & solver)
{
  for (auto & armor : armors) solver.solve(armor);
}

std::optional<UsbCandidate> get_nearest_candidate(
  const std::list<auto_aim::Armor> & armors, std::chrono::steady_clock::time_point timestamp,
  TargetSource source)
{
  if (armors.empty()) return std::nullopt;

  auto nearest = armors.front();
  auto min_distance = nearest.xyz_in_gimbal.norm();
  for (const auto & armor : armors) {
    auto distance = armor.xyz_in_gimbal.norm();
    if (distance < min_distance) {
      min_distance = distance;
      nearest = armor;
    }
  }

  return UsbCandidate{nearest, timestamp, source, min_distance};
}

std::optional<UsbCandidate> select_nearest_candidate(
  const std::list<auto_aim::Armor> & left_armors, std::chrono::steady_clock::time_point left_t,
  const std::list<auto_aim::Armor> & right_armors, std::chrono::steady_clock::time_point right_t)
{
  auto left_candidate = get_nearest_candidate(left_armors, left_t, TargetSource::usb_left);
  auto right_candidate = get_nearest_candidate(right_armors, right_t, TargetSource::usb_right);

  if (!left_candidate.has_value()) return right_candidate;
  if (!right_candidate.has_value()) return left_candidate;

  return left_candidate->distance <= right_candidate->distance ? left_candidate : right_candidate;
}

}  // namespace

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{display d      |                        | 显示主相机与左右USB画面}"
  "{disable-usb    |                        | 禁用左右USB侧视相机，仅使用主海康相机}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  bool display = cli.has("display");
  const bool use_usb = !cli.has("disable-usb");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  auto yaml = tools::load(config_path);
  auto enemy_color =
    tools::read<std::string>(yaml, "enemy_color") == "red" ? auto_aim::Color::red : auto_aim::Color::blue;

  io::Gimbal gimbal(config_path);
  io::ROS2 ros2;
  std::unique_ptr<io::USBCamera> usb_left_camera;
  std::unique_ptr<io::USBCamera> usb_right_camera;
  if (use_usb) {
    usb_left_camera = std::make_unique<io::USBCamera>(USB_LEFT_DEVICE, config_path);
    usb_right_camera = std::make_unique<io::USBCamera>(USB_RIGHT_DEVICE, config_path);
    // Match `usbcamera_test` startup more closely: let the USB cameras settle before starting HikRobot.
    std::this_thread::sleep_for(USB_STARTUP_SETTLE);
  } else {
    tools::logger()->info("[sentry_omni_perception_debug_mpc] USB side cameras disabled.");
  }
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, display);
  auto_aim::YOLO usb_yolo(config_path, false);
  auto_buff::YOLO11_BUFF big_buff_yolo(config_path);

  auto_aim::Solver main_solver(config_path);
  auto_aim::Solver usb_left_solver(config_path);
  auto_aim::Solver usb_right_solver(config_path);
  auto_buff::Solver big_buff_solver(config_path);

  auto_aim::Tracker main_tracker(config_path, main_solver);
  auto_aim::Tracker usb_left_tracker(config_path, usb_left_solver);
  auto_aim::Tracker usb_right_tracker(config_path, usb_right_solver);
  auto_aim::Planner planner(config_path);
  auto_buff::Aimer big_buff_aimer(config_path);
  std::vector<BigBuffTrack> big_buff_tracks;
  std::optional<int> locked_big_buff_track_id = std::nullopt;
  int next_big_buff_track_id = 1;

  tools::ThreadSafeQueue<TargetCommand, true> target_queue(1);
  target_queue.push(TargetCommand{});

  std::atomic<bool> quit = false;
  std::atomic<bool> main_camera_has_target = false;
  std::mutex usb_result_mutex;
  UsbThreadResult usb_result;

  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      if (gimbal.mode() == io::GimbalMode::BIG_BUFF) {
        std::this_thread::sleep_for(10ms);
        continue;
      }

      auto target_command = target_queue.front();
      auto gs = gimbal.state();
      auto plan = planner.plan(target_command.target, gs.bullet_speed);

      const bool use_yaw_diff =
        target_command.source == TargetSource::usb_left || target_command.source == TargetSource::usb_right;
      if (use_yaw_diff && plan.control) {
        auto yaw_offset =
          gs.yaw_diff + usb_yaw_offset(target_command.source) + usb_target_yaw_trim(target_command.source);
        plan.target_yaw = tools::limit_rad(plan.target_yaw + yaw_offset);
        plan.yaw = tools::limit_rad(plan.yaw + yaw_offset);
        plan.target_pitch += USB_TARGET_PITCH_TRIM;
        plan.pitch += USB_TARGET_PITCH_TRIM;
      }

      const bool fire = target_command.source == TargetSource::main && plan.fire;
      gimbal.send(
        plan.control, fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;
      data["yaw_diff"] = gs.yaw_diff;
      data["usb_yaw_offset"] = usb_yaw_offset(target_command.source);
      data["usb_target_yaw_trim"] = usb_target_yaw_trim(target_command.source);
      data["usb_target_pitch_trim"] = use_yaw_diff ? USB_TARGET_PITCH_TRIM : 0.0;
      data["active_source"] = static_cast<int>(target_command.source);
      data["use_yaw_diff"] = use_yaw_diff ? 1 : 0;

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      if (target_command.target.has_value()) {
        data["target_z"] = target_command.target->ekf_x()[4];
        data["target_vz"] = target_command.target->ekf_x()[5];
        data["w"] = target_command.target->ekf_x()[7];
      } else {
        data["w"] = 0.0;
      }

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  auto ros_thread = std::thread([&]() {
    while (!quit) {
      ros2.publish(gimbal.game_status());
      ros2.publish(gimbal.event_data());
      ros2.publish(gimbal.robot_status());
      ros2.publish(gimbal.hurt_data());
      ros2.publish(gimbal.sentry_info());
      ros2.publish(gimbal.rfid_status());
      ros2.publish(gimbal.robot_pos());
      ros2.publish(gimbal.ground_robot_pos());
      ros2.publish(gimbal.game_robot_hp());
      auto gs = gimbal.state();
      ros2.publish(gs.yaw, gs.pitch, gs.yaw_diff);

      ros2.spin_some();

      gimbal.send(
        ros2.getTargetMode(), ros2.getChassisStatus(), ros2.getSentryStatus(), ros2.getCmdVelX(),
        ros2.getCmdVelY(), ros2.getCmdVelZ(), ros2.getTerrainStatus(), ros2.getBumpStatus());
      std::this_thread::sleep_for(20ms);
    }
  });

  std::thread usb_thread;
  if (use_usb) {
    usb_thread = std::thread([&]() {
      while (!quit) {
        const auto mode = gimbal.mode();
        cv::Mat usb_left_img;
        cv::Mat usb_right_img;
        std::chrono::steady_clock::time_point usb_left_t;
        std::chrono::steady_clock::time_point usb_right_t;

        usb_left_camera->read(usb_left_img, usb_left_t);
        if (quit) break;
        usb_right_camera->read(usb_right_img, usb_right_t);
        if (quit) break;

        auto usb_left_q = usb_world_q(gimbal.q(usb_left_t - GIMBAL_DELAY));
        auto usb_right_q = usb_world_q(gimbal.q(usb_right_t - GIMBAL_DELAY));
        usb_left_solver.set_R_gimbal2world(usb_left_q);
        usb_right_solver.set_R_gimbal2world(usb_right_q);

        UsbThreadResult next_usb_result;
        next_usb_result.left_img = usb_left_img;
        next_usb_result.right_img = usb_right_img;
        if (main_camera_has_target.load()) {
          next_usb_result.ready = true;
          std::lock_guard<std::mutex> lock(usb_result_mutex);
          usb_result = std::move(next_usb_result);
          std::this_thread::sleep_for(10ms);
          continue;
        }

        next_usb_result.left_armors =
          filter_armors(usb_yolo.detect(usb_left_img), enemy_color, mode);
        next_usb_result.right_armors =
          filter_armors(usb_yolo.detect(usb_right_img), enemy_color, mode);

        solve_armors(next_usb_result.left_armors, usb_left_solver);
        solve_armors(next_usb_result.right_armors, usb_right_solver);
        next_usb_result.candidate = select_nearest_candidate(
          next_usb_result.left_armors, usb_left_t, next_usb_result.right_armors, usb_right_t);

        if (next_usb_result.candidate.has_value()) {
          std::list<auto_aim::Armor> selected_armors = {next_usb_result.candidate->armor};
          std::list<auto_aim::Target> usb_targets;

          if (next_usb_result.candidate->source == TargetSource::usb_left) {
            usb_targets = usb_left_tracker.track(selected_armors, next_usb_result.candidate->timestamp, false);
          } else {
            usb_targets =
              usb_right_tracker.track(selected_armors, next_usb_result.candidate->timestamp, false);
          }

          if (!usb_targets.empty()) {
            next_usb_result.target_command.target = usb_targets.front();
            next_usb_result.target_command.source = next_usb_result.candidate->source;
          }
        }

        next_usb_result.ready = true;
        std::lock_guard<std::mutex> lock(usb_result_mutex);
        usb_result = std::move(next_usb_result);
      }
    });
  }

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);

    auto q = gimbal.q(t - GIMBAL_DELAY);
    auto gs = gimbal.state();
    auto mode = gimbal.mode();
    auto q_ypr = tools::eulers(q, 2, 1, 0);

    std::list<auto_aim::Armor> main_armors;
    std::list<auto_aim::Target> main_targets;
    std::vector<BigBuffCandidate> big_buff_candidates;
    auto_aim::Plan big_buff_plan{false, false, 0, 0, 0, 0, 0, 0, 0, 0};

    UsbThreadResult current_usb_result;
    TargetCommand target_command;

    if (mode == io::GimbalMode::BIG_BUFF) {
      target_queue.push(TargetCommand{});
      main_camera_has_target = false;

      big_buff_solver.set_R_gimbal2world(q);
      big_buff_candidates = filter_big_buff_candidates(big_buff_yolo.get_multicandidateboxes(img));
      update_big_buff_tracks(
        big_buff_candidates, big_buff_tracks, locked_big_buff_track_id, next_big_buff_track_id);

      auto * locked_track = find_big_buff_track(big_buff_tracks, locked_big_buff_track_id);
      if (locked_track != nullptr && locked_track->power_rune.has_value()) {
        big_buff_solver.solve(locked_track->power_rune);
        locked_track->target.get_target(locked_track->power_rune, t);
        auto target_copy = locked_track->target;
        big_buff_plan = big_buff_aimer.mpc_aim(target_copy, t, gs, true);
      }

      gimbal.send(
        big_buff_plan.control, big_buff_plan.fire, big_buff_plan.yaw, big_buff_plan.yaw_vel,
        big_buff_plan.yaw_acc, big_buff_plan.pitch, big_buff_plan.pitch_vel,
        big_buff_plan.pitch_acc);
    } else {
      big_buff_tracks.clear();
      locked_big_buff_track_id = std::nullopt;

      main_solver.set_R_gimbal2world(q);
      main_armors = filter_armors(yolo.detect(img), enemy_color, mode);
      solve_armors(main_armors, main_solver);
      main_camera_has_target = !main_armors.empty();
      main_targets = main_tracker.track(main_armors, t);

      {
        std::lock_guard<std::mutex> lock(usb_result_mutex);
        current_usb_result = usb_result;
      }

      if (!main_targets.empty()) {
        target_command.target = main_targets.front();
        target_command.source = TargetSource::main;
      } else if (current_usb_result.ready) {
        target_command = current_usb_result.target_command;
      }

      target_queue.push(target_command);
    }

    ros2.publish(make_target_msg(target_command));

    nlohmann::json data;
    data["q_yaw"] = q_ypr[0];
    data["q_pitch"] = q_ypr[1];
    data["q_roll"] = q_ypr[2];
    data["gimbal_yaw"] = gs.yaw;
    data["gimbal_pitch"] = gs.pitch;
    data["gimbal_yaw_err"] = tools::limit_rad(q_ypr[0] - gs.yaw);
    data["gimbal_pitch_err"] = q_ypr[1] - gs.pitch;
    data["main_armor_num"] = main_armors.size();
    data["usb_left_armor_num"] = current_usb_result.left_armors.size();
    data["usb_right_armor_num"] = current_usb_result.right_armors.size();
    data["active_source"] = static_cast<int>(target_command.source);
    data["big_buff_candidate_num"] = big_buff_candidates.size();
    data["big_buff_track_num"] = big_buff_tracks.size();
    data["big_buff_locked_id"] = locked_big_buff_track_id.value_or(-1);
    data["big_buff_control"] = big_buff_plan.control ? 1 : 0;
    data["big_buff_fire"] = big_buff_plan.fire ? 1 : 0;

    if (!main_armors.empty()) {
      const auto & armor = main_armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_z"] = armor.xyz_in_world[2];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
      data["armor_center_x"] = armor.center_norm.x;
      data["armor_center_y"] = armor.center_norm.y;
    }

    if (current_usb_result.candidate.has_value()) {
      data["usb_candidate_distance"] = current_usb_result.candidate->distance;
      data["usb_candidate_yaw"] = current_usb_result.candidate->armor.ypd_in_world[0];
      data["usb_candidate_pitch"] = current_usb_result.candidate->armor.ypd_in_world[1];
      data["usb_candidate_source"] = static_cast<int>(current_usb_result.candidate->source);
    }

    if (locked_big_buff_track_id.has_value()) {
      data["big_buff_target_yaw"] = big_buff_plan.yaw;
      data["big_buff_target_pitch"] = big_buff_plan.pitch;
    }

    plotter.plot(data);

    if (target_command.source == TargetSource::main && !main_targets.empty()) {
      auto target = main_targets.front();

      for (const auto & xyza : target.armor_xyza_list()) {
        auto image_points =
          main_solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        main_solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});
    }

    for (const auto & track : big_buff_tracks) {
      const auto color = locked_big_buff_track_id.has_value() && track.id == locked_big_buff_track_id.value()
                           ? cv::Scalar{0, 0, 255}
                           : cv::Scalar{0, 255, 0};
      cv::circle(img, track.center, 5, color, -1, cv::LINE_AA);
      tools::draw_text(
        img, fmt::format("buff#{} {:.2f}", track.id, track.confidence),
        track.center + cv::Point2f{6.0F, -6.0F}, color);
    }

    tools::draw_text(
      img,
      fmt::format(
        "[{}] main:{} usb_l:{} usb_r:{} buff:{}",
        source_name(target_command.source), main_armors.size(), current_usb_result.left_armors.size(),
        current_usb_result.right_armors.size(), big_buff_tracks.size()),
      {10, 30}, {255, 255, 255});

    if (display) {
      try {
        if (current_usb_result.ready) {
          if (!current_usb_result.left_img.empty()) {
            auto usb_left_preview = current_usb_result.left_img.clone();
            for (const auto & armor : current_usb_result.left_armors) {
              tools::draw_points(usb_left_preview, armor.points, {0, 255, 0});
            }
            if (
              current_usb_result.candidate.has_value() &&
              current_usb_result.candidate->source == TargetSource::usb_left)
            {
              tools::draw_points(
                usb_left_preview, current_usb_result.candidate->armor.points, {0, 0, 255});
            }
            tools::draw_text(
              usb_left_preview,
              fmt::format(
                "[usb_left{}] armors:{}",
                target_command.source == TargetSource::usb_left ? "*" : "",
                current_usb_result.left_armors.size()),
              {10, 30}, {255, 255, 255});
            cv::resize(usb_left_preview, usb_left_preview, {}, 0.5, 0.5);
            cv::imshow("usb_left", usb_left_preview);
          }

          if (!current_usb_result.right_img.empty()) {
            auto usb_right_preview = current_usb_result.right_img.clone();
            for (const auto & armor : current_usb_result.right_armors) {
              tools::draw_points(usb_right_preview, armor.points, {0, 255, 0});
            }
            if (
              current_usb_result.candidate.has_value() &&
              current_usb_result.candidate->source == TargetSource::usb_right)
            {
              tools::draw_points(
                usb_right_preview, current_usb_result.candidate->armor.points, {0, 0, 255});
            }
            tools::draw_text(
              usb_right_preview,
              fmt::format(
                "[usb_right{}] armors:{}",
                target_command.source == TargetSource::usb_right ? "*" : "",
                current_usb_result.right_armors.size()),
              {10, 30}, {255, 255, 255});
            cv::resize(usb_right_preview, usb_right_preview, {}, 0.5, 0.5);
            cv::imshow("usb_right", usb_right_preview);
          }
        }

        auto main_preview = img.clone();
        cv::resize(main_preview, main_preview, {}, 0.5, 0.5);
        cv::imshow("reprojection", main_preview);
        auto key = cv::waitKey(1);
        if (key == 'q') break;
      } catch (const cv::Exception & e) {
        tools::logger()->warn("OpenCV GUI unavailable, disable display windows: {}", e.what());
        display = false;
      }
    }
  }

  quit = true;
  if (ros_thread.joinable()) ros_thread.join();
  if (usb_thread.joinable()) usb_thread.join();
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  if (display) cv::destroyAllWindows();

  return 0;
}
