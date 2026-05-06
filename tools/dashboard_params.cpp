#include "tools/dashboard_params.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tools/dashboard_mqtt_contract.hpp"
#include "tools/yaml.hpp"

namespace tools
{
namespace dashboard
{
namespace
{
using ParamSpec = DashboardParams::ParamSpec;

constexpr char STANDARD3_CONFIG_PATH[] = "configs/standard3.yaml";

DashboardParamResult make_result(
  bool ok, DashboardParamStatus status, const std::string & key, const std::string & message,
  nlohmann::json applied = nlohmann::json::object())
{
  return {ok, status, key, message, applied};
}

ParamSpec make_number_spec(
  const std::string & key, const std::string & local_key,
  double DashboardParamSnapshot::*snapshot_value, double min, double max, double step,
  const std::string & unit, const std::string & group)
{
  ParamSpec spec;
  spec.key = key;
  spec.local_key = local_key;
  spec.group = group;
  spec.type = "number";
  spec.snapshot_value = snapshot_value;
  spec.editable = true;
  spec.restart_required = false;
  spec.has_min = true;
  spec.has_max = true;
  spec.has_step = true;
  spec.min = min;
  spec.max = max;
  spec.step = step;
  spec.unit = unit;
  return spec;
}

const std::vector<ParamSpec> & legacy_param_specs()
{
  static const std::vector<ParamSpec> specs{
    make_number_spec(
      "planner.yaw_offset_deg", "yaw_offset_deg", &DashboardParamSnapshot::planner_yaw_offset_deg,
      -20.0, 20.0, 0.1, "deg", "planner"),
    make_number_spec(
      "planner.pitch_offset_deg", "pitch_offset_deg",
      &DashboardParamSnapshot::planner_pitch_offset_deg, -20.0, 20.0, 0.1, "deg", "planner"),
    make_number_spec(
      "planner.fire_thresh", "fire_thresh", &DashboardParamSnapshot::planner_fire_thresh, 0.0,
      0.05, 0.0001, "rad", "planner"),
    make_number_spec(
      "planner.decision_speed", "decision_speed", &DashboardParamSnapshot::planner_decision_speed,
      0.0, 30.0, 0.1, "rad/s", "planner"),
    make_number_spec(
      "planner.high_speed_delay_time", "high_speed_delay_time",
      &DashboardParamSnapshot::planner_high_speed_delay_time, 0.0, 0.5, 0.001, "s", "planner"),
    make_number_spec(
      "planner.low_speed_delay_time", "low_speed_delay_time",
      &DashboardParamSnapshot::planner_low_speed_delay_time, 0.0, 0.5, 0.001, "s", "planner")};
  return specs;
}

bool spec_enabled(const ParamSpec & spec, bool include_buff)
{
  return include_buff || (spec.group != "buff" && spec.group != "buff_aimer" &&
                          spec.group != "buff_detector" &&
                          spec.key.rfind("buff.", 0) != 0 &&
                          spec.key.rfind("buff_aimer.", 0) != 0);
}

const ParamSpec * find_spec(
  const std::vector<ParamSpec> & specs, const std::string & key, bool include_buff)
{
  const auto iter = std::find_if(specs.begin(), specs.end(), [&key, include_buff](const auto & spec) {
    return key == spec.key && spec_enabled(spec, include_buff);
  });
  return iter == specs.end() ? nullptr : &*iter;
}

std::string group_for_key(const std::string & key)
{
  if (key == "enemy_color" || key == "yolo_name" || key == "classify_model" ||
      key == "yolo11_model_path" || key == "yolov8_model_path" || key == "yolov5_model_path" ||
      key == "device" || key == "min_confidence" || key == "use_traditional") {
    return "vision";
  }
  if (key == "roi" || key.rfind("roi.", 0) == 0 || key == "use_roi") {
    return "roi";
  }
  if (key == "threshold" || key == "max_angle_error" || key == "min_lightbar_ratio" ||
      key == "max_lightbar_ratio" || key == "min_lightbar_length" || key == "min_armor_ratio" ||
      key == "max_armor_ratio" || key == "max_side_ratio" || key == "max_rectangular_error") {
    return "traditional";
  }
  if (key == "min_detect_count" || key == "max_temp_lost_count" ||
      key == "outpost_max_temp_lost_count") {
    return "tracker";
  }
  if (key == "yaw_offset" || key == "pitch_offset" || key == "comming_angle" ||
      key == "leaving_angle" || key == "decision_speed" || key == "high_speed_delay_time" ||
      key == "low_speed_delay_time") {
    return "aimer";
  }
  if (key == "first_tolerance" || key == "second_tolerance" || key == "judge_distance" ||
      key == "auto_fire") {
    return "shooter";
  }
  if (key == "camera_name" || key == "exposure_ms" || key == "gain" || key == "vid_pid") {
    return "camera";
  }
  if (key == "R_gimbal2imubody" || key == "camera_matrix" || key == "distort_coeffs" ||
      key == "R_camera2gimbal" || key == "t_camera2gimbal") {
    return "calibration";
  }
  if (key == "quaternion_canid" || key == "bullet_speed_canid" || key == "send_canid" ||
      key == "can_interface") {
    return "cboard";
  }
  if (key == "com_port" || key == "yaw_kp" || key == "yaw_kd" || key == "pitch_kp" ||
      key == "pitch_kd") {
    return "gimbal";
  }
  if (key == "fire_thresh" || key == "max_yaw_acc" || key == "Q_yaw" || key == "R_yaw" ||
      key == "max_pitch_acc" || key == "Q_pitch" || key == "R_pitch") {
    return "planner";
  }
  if (key == "model") {
    return "buff_detector";
  }
  if (key == "fire_gap_time" || key == "predict_time") {
    return "buff_aimer";
  }
  return "config";
}

nlohmann::json yaml_to_json(const YAML::Node & node)
{
  if (!node) {
    return nullptr;
  }
  if (node.IsScalar()) {
    const auto value = node.as<std::string>();
    if (value == "true") {
      return true;
    }
    if (value == "false") {
      return false;
    }
    try {
      std::size_t parsed = 0;
      const auto number = std::stoll(value, &parsed, 0);
      if (parsed == value.size()) {
        return number;
      }
    } catch (const std::exception &) {
    }
    try {
      std::size_t parsed = 0;
      const auto number = std::stod(value, &parsed);
      if (parsed == value.size()) {
        return number;
      }
    } catch (const std::exception &) {
    }
    return value;
  }
  if (node.IsSequence()) {
    auto array = nlohmann::json::array();
    for (const auto & item : node) {
      array.push_back(yaml_to_json(item));
    }
    return array;
  }
  if (node.IsMap()) {
    auto object = nlohmann::json::object();
    for (const auto & item : node) {
      object[item.first.as<std::string>()] = yaml_to_json(item.second);
    }
    return object;
  }
  return nullptr;
}

std::string flatten_key(const std::string & prefix, const std::string & key)
{
  return prefix.empty() ? key : prefix + "." + key;
}

void add_yaml_specs(
  const YAML::Node & node, const std::string & prefix, const std::string & forced_group,
  std::vector<ParamSpec> & specs)
{
  if (!node.IsMap()) {
    return;
  }

  for (const auto & item : node) {
    const auto yaml_key = item.first.as<std::string>();
    const auto key = flatten_key(prefix, yaml_key);
    const auto & value = item.second;
    const auto group = forced_group.empty() ? group_for_key(key) : forced_group;

    if (value.IsMap()) {
      add_yaml_specs(value, key, group, specs);
      continue;
    }

    ParamSpec spec;
    spec.key = key;
    spec.local_key = key;
    spec.group = group;
    spec.value = yaml_to_json(value);
    spec.editable = false;
    spec.restart_required = true;

    if (value.IsScalar()) {
      if (spec.value.is_boolean()) {
        spec.type = "bool";
      } else if (spec.value.is_number()) {
        spec.type = "number";
      } else {
        spec.type = "string";
      }
    } else {
      spec.type = "string";
      spec.value = yaml_to_json(value).dump();
      spec.render_json = true;
    }
    specs.push_back(std::move(spec));
  }
}

ParamSpec * find_mutable_spec(std::vector<ParamSpec> & specs, const std::string & key)
{
  const auto iter =
    std::find_if(specs.begin(), specs.end(), [&key](const auto & spec) { return spec.key == key; });
  return iter == specs.end() ? nullptr : &*iter;
}

void make_hot(
  std::vector<ParamSpec> & specs, const std::string & config_key, const std::string & dashboard_key,
  const std::string & local_key, double DashboardParamSnapshot::*snapshot_value, double min,
  double max, double step, const std::string & unit, const std::string & group)
{
  auto * spec = find_mutable_spec(specs, config_key);
  if (spec == nullptr) {
    return;
  }
  spec->key = dashboard_key;
  spec->local_key = local_key;
  spec->group = group;
  spec->type = "number";
  spec->snapshot_value = snapshot_value;
  spec->editable = true;
  spec->restart_required = false;
  spec->has_min = true;
  spec->has_max = true;
  spec->has_step = true;
  spec->min = min;
  spec->max = max;
  spec->step = step;
  spec->unit = unit;
}

std::vector<ParamSpec> make_standard3_specs(const std::string & config_path, bool include_buff)
{
  std::vector<ParamSpec> specs;
  add_yaml_specs(tools::load(config_path), "", "", specs);

  make_hot(
    specs, "yaw_offset", "aimer.yaw_offset", "yaw_offset_deg",
    &DashboardParamSnapshot::planner_yaw_offset_deg, -20.0, 20.0, 0.1, "deg", "aimer");
  make_hot(
    specs, "pitch_offset", "aimer.pitch_offset", "pitch_offset_deg",
    &DashboardParamSnapshot::planner_pitch_offset_deg, -20.0, 20.0, 0.1, "deg", "aimer");
  make_hot(
    specs, "decision_speed", "aimer.decision_speed", "decision_speed",
    &DashboardParamSnapshot::planner_decision_speed, 0.0, 30.0, 0.1, "rad/s", "aimer");
  make_hot(
    specs, "high_speed_delay_time", "aimer.high_speed_delay_time", "high_speed_delay_time",
    &DashboardParamSnapshot::planner_high_speed_delay_time, 0.0, 0.5, 0.001, "s", "aimer");
  make_hot(
    specs, "low_speed_delay_time", "aimer.low_speed_delay_time", "low_speed_delay_time",
    &DashboardParamSnapshot::planner_low_speed_delay_time, 0.0, 0.5, 0.001, "s", "aimer");
  make_hot(
    specs, "fire_thresh", "planner.fire_thresh", "fire_thresh",
    &DashboardParamSnapshot::planner_fire_thresh, 0.0, 0.05, 0.0001, "rad", "planner");

  return specs;
}

nlohmann::json spec_value(const ParamSpec & spec, const DashboardParamSnapshot & snapshot)
{
  if (spec.snapshot_value != nullptr) {
    return snapshot.*(spec.snapshot_value);
  }
  return spec.value;
}

nlohmann::json make_schema_item(const ParamSpec & spec, const DashboardParamSnapshot & snapshot)
{
  nlohmann::json item{
    {"key", spec.key},
    {"type", spec.type},
    {"group", spec.group},
    {"value", spec_value(spec, snapshot)},
    {"editable", spec.editable},
    {"restart_required", spec.restart_required}};

  if (spec.has_min) {
    item["min"] = spec.min;
  }
  if (spec.has_max) {
    item["max"] = spec.max;
  }
  if (spec.has_step) {
    item["step"] = spec.step;
  }
  if (!spec.unit.empty()) {
    item["unit"] = spec.unit;
  }
  if (!spec.options.empty()) {
    item["options"] = spec.options;
  }
  if (spec.render_json) {
    item["render"] = "json";
  }
  if (!spec.source_key.empty()) {
    item["source_key"] = spec.source_key;
  }
  return item;
}

}  // namespace

DashboardParams::DashboardParams(
  SnapshotReader read_snapshot, ParamWriter write_param, bool include_buff)
: read_snapshot_(std::move(read_snapshot)),
  write_param_(std::move(write_param)),
  include_buff_(include_buff),
  params_(legacy_param_specs())
{
  if (!read_snapshot_ || !write_param_) {
    throw std::invalid_argument("DashboardParams requires read and write callbacks");
  }
}

DashboardParams::DashboardParams(auto_aim::Planner & planner)
: DashboardParams(STANDARD3_CONFIG_PATH, planner, false)
{
}

DashboardParams::DashboardParams(
  const std::string & config_path, auto_aim::Planner & planner, bool include_buff)
: DashboardParams(
    [&planner]() {
      const auto planner_params = planner.get_hot_params();
      return DashboardParamSnapshot{
        planner_params.yaw_offset_deg,
        planner_params.pitch_offset_deg,
        planner_params.fire_thresh,
        planner_params.decision_speed,
        planner_params.high_speed_delay_time,
        planner_params.low_speed_delay_time};
    },
    [&planner](const DashboardParamUpdate & update) {
      return planner.apply_hot_param(update.local_key, update.value);
    },
    include_buff)
{
  params_ = make_standard3_specs(config_path, include_buff_);
}

nlohmann::json DashboardParams::make_schema() const
{
  const auto snapshot = read_snapshot_();
  std::vector<nlohmann::json> params;
  params.reserve(params_.size());
  for (const auto & spec : params_) {
    if (!spec_enabled(spec, include_buff_)) {
      continue;
    }
    params.push_back(make_schema_item(spec, snapshot));
  }
  return make_params_schema_payload(params);
}

nlohmann::json DashboardParams::make_current(std::int64_t timestamp) const
{
  const auto snapshot = read_snapshot_();
  nlohmann::json values = nlohmann::json::object();
  for (const auto & spec : params_) {
    if (!spec_enabled(spec, include_buff_)) {
      continue;
    }
    values[spec.key] = spec_value(spec, snapshot);
  }
  return make_params_current_payload(values, timestamp);
}

DashboardParamResult DashboardParams::apply(const std::string & key, const nlohmann::json & value)
  const
{
  const auto * spec = find_spec(params_, key, include_buff_);
  if (spec == nullptr) {
    return make_result(false, DashboardParamStatus::UnknownKey, key, "unknown parameter key");
  }

  if (!spec->editable) {
    return make_result(false, DashboardParamStatus::ApplyFailed, key, "parameter is read-only");
  }

  if (spec->type != "number" || !value.is_number() || value.is_boolean()) {
    return make_result(false, DashboardParamStatus::TypeError, key, "parameter value must be number");
  }

  const auto number = value.get<double>();
  if (!std::isfinite(number)) {
    return make_result(false, DashboardParamStatus::TypeError, key, "parameter value must be finite");
  }

  if ((spec->has_min && number < spec->min) || (spec->has_max && number > spec->max)) {
    return make_result(false, DashboardParamStatus::OutOfRange, key, "parameter value out of range");
  }

  DashboardParamUpdate update{spec->key, spec->local_key, number};
  if (!write_param_(update)) {
    return make_result(false, DashboardParamStatus::ApplyFailed, key, "parameter apply failed");
  }

  return make_result(
    true, DashboardParamStatus::Applied, key, spec->key + " applied",
    nlohmann::json{{spec->key, number}});
}

}  // namespace dashboard
}  // namespace tools
