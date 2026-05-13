#include <fmt/core.h>

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <list>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>

#include "combat_rm_interfaces/msg/armor.hpp"
#include "combat_rm_interfaces/msg/armors.hpp"
#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "io/ros2/ros2.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
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

combat_rm_interfaces::msg::Armors make_armors_msg(const std::list<auto_aim::Armor> & armors)
{
  combat_rm_interfaces::msg::Armors msg;
  msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  msg.header.frame_id = "vision_world";
  msg.armors.reserve(armors.size());

  for (const auto & armor : armors) {
    combat_rm_interfaces::msg::Armor armor_msg;
    armor_msg.number = armor_number(armor.name);
    armor_msg.type = auto_aim::ARMOR_TYPES[armor.type];
    armor_msg.pose.position.x = armor.xyz_in_world.x();
    armor_msg.pose.position.y = armor.xyz_in_world.y();
    armor_msg.pose.position.z = armor.xyz_in_world.z();

    const Eigen::Quaterniond q =
      Eigen::AngleAxisd(armor.ypr_in_world[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(armor.ypr_in_world[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(armor.ypr_in_world[2], Eigen::Vector3d::UnitX());
    armor_msg.pose.orientation.x = q.x();
    armor_msg.pose.orientation.y = q.y();
    armor_msg.pose.orientation.z = q.z();
    armor_msg.pose.orientation.w = q.w();

    msg.armors.emplace_back(std::move(armor_msg));
  }

  return msg;
}

Eigen::Quaterniond usb_world_q(const Eigen::Quaterniond & q)
{
  // The side USB cameras do not follow gimbal pitch; keep yaw/roll and clamp pitch to zero.
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

  auto_aim::Solver main_solver(config_path);
  auto_aim::Solver usb_left_solver(config_path);
  auto_aim::Solver usb_right_solver(config_path);

  auto_aim::Tracker main_tracker(config_path, main_solver);
  auto_aim::Tracker usb_left_tracker(config_path, usb_left_solver);
  auto_aim::Tracker usb_right_tracker(config_path, usb_right_solver);
  auto_aim::Planner planner(config_path);

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
        0, ros2.getChassisStatus(), ros2.getSentryStatus(), ros2.getCmdVelX(), ros2.getCmdVelY());
      std::this_thread::sleep_for(20ms);
    }
  });

  std::thread usb_thread;
  if (use_usb) {
    usb_thread = std::thread([&]() {
      while (!quit) {
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
          filter_enemy_armors(usb_yolo.detect(usb_left_img), enemy_color);
        next_usb_result.right_armors =
          filter_enemy_armors(usb_yolo.detect(usb_right_img), enemy_color);

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
    auto q_ypr = tools::eulers(q, 2, 1, 0);

    main_solver.set_R_gimbal2world(q);
    auto main_armors = yolo.detect(img);
    solve_armors(main_armors, main_solver);
    main_camera_has_target = !main_armors.empty();
    auto main_targets = main_tracker.track(main_armors, t);
    ros2.publish(make_armors_msg(main_armors));

    UsbThreadResult current_usb_result;
    {
      std::lock_guard<std::mutex> lock(usb_result_mutex);
      current_usb_result = usb_result;
    }

    TargetCommand target_command;

    if (!main_targets.empty()) {
      target_command.target = main_targets.front();
      target_command.source = TargetSource::main;
    } else if (current_usb_result.ready) {
      target_command = current_usb_result.target_command;
    }

    target_queue.push(target_command);

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

    tools::draw_text(
      img,
      fmt::format(
        "[{}] main:{} usb_l:{} usb_r:{}",
        source_name(target_command.source), main_armors.size(), current_usb_result.left_armors.size(),
        current_usb_result.right_armors.size()),
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
