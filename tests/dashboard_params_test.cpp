#include "tools/dashboard_params.hpp"
#include "tools/dashboard_mqtt_contract.hpp"

#include <cassert>
#include <cmath>
#include <string>

namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

bool close_to(double lhs, double rhs)
{
  return std::abs(lhs - rhs) < 1e-9;
}

}  // namespace

int main()
{
  struct State
  {
    double planner_yaw_offset_rad = 1.0 * DEG_TO_RAD;
    double planner_pitch_offset_rad = -2.0 * DEG_TO_RAD;
    double planner_fire_thresh = 0.0035;
    double planner_decision_speed = 7.0;
    double planner_high_speed_delay_time = 0.065;
    double planner_low_speed_delay_time = 0.052;
  } state;

  auto read_snapshot = [&state]() {
    return tools::dashboard::DashboardParamSnapshot{
      state.planner_yaw_offset_rad * RAD_TO_DEG,
      state.planner_pitch_offset_rad * RAD_TO_DEG,
      state.planner_fire_thresh,
      state.planner_decision_speed,
      state.planner_high_speed_delay_time,
      state.planner_low_speed_delay_time};
  };

  auto write_param = [&state](const tools::dashboard::DashboardParamUpdate & update) {
    if (update.key == "planner.yaw_offset_deg") {
      state.planner_yaw_offset_rad = update.value * DEG_TO_RAD;
    } else if (update.key == "planner.pitch_offset_deg") {
      state.planner_pitch_offset_rad = update.value * DEG_TO_RAD;
    } else if (update.key == "planner.fire_thresh") {
      state.planner_fire_thresh = update.value;
    } else if (update.key == "planner.decision_speed") {
      state.planner_decision_speed = update.value;
    } else if (update.key == "planner.high_speed_delay_time") {
      state.planner_high_speed_delay_time = update.value;
    } else if (update.key == "planner.low_speed_delay_time") {
      state.planner_low_speed_delay_time = update.value;
    } else {
      return false;
    }
    return update.local_key.find('.') == std::string::npos;
  };

  tools::dashboard::DashboardParams params(read_snapshot, write_param, true);

  const auto schema = params.make_schema();
  assert(schema.at("version") == 1);
  assert(schema.at("params").size() == 6);

  const auto current = params.make_current(1234);
  assert(current.at("timestamp") == 1234);
  assert(close_to(current.at("values").at("planner.yaw_offset_deg").get<double>(), 1.0));

  const auto applied = params.apply("planner.fire_thresh", 0.01);
  assert(applied.ok);
  assert(applied.status == tools::dashboard::DashboardParamStatus::Applied);
  assert(close_to(state.planner_fire_thresh, 0.01));
  assert(close_to(applied.applied.at("planner.fire_thresh").get<double>(), 0.01));

  const auto out_of_range = params.apply("planner.fire_thresh", 1.0);
  assert(!out_of_range.ok);
  assert(out_of_range.status == tools::dashboard::DashboardParamStatus::OutOfRange);
  assert(close_to(state.planner_fire_thresh, 0.01));

  const auto unknown = params.apply("planner.max_yaw_acc", 50.0);
  assert(!unknown.ok);
  assert(unknown.status == tools::dashboard::DashboardParamStatus::UnknownKey);

  const auto type_error = params.apply("planner.decision_speed", "0.2");
  assert(!type_error.ok);
  assert(type_error.status == tools::dashboard::DashboardParamStatus::TypeError);

  const auto planner_deg = params.apply("planner.yaw_offset_deg", 10.0);
  assert(planner_deg.ok);
  assert(close_to(state.planner_yaw_offset_rad, 10.0 * DEG_TO_RAD));
  assert(close_to(
    params.make_current(1235).at("values").at("planner.yaw_offset_deg").get<double>(), 10.0));

  tools::dashboard::DashboardParams planner_only(read_snapshot, write_param, false);
  const auto planner_only_schema = planner_only.make_schema();
  assert(planner_only_schema.at("params").size() == 6);
  for (const auto & item : planner_only_schema.at("params")) {
    const auto key = item.at("key").get<std::string>();
    assert(key.rfind("planner.", 0) == 0);
  }

  const auto planner_only_current = planner_only.make_current(2234);
  assert(planner_only_current.at("timestamp") == 2234);
  assert(planner_only_current.at("values").contains("planner.fire_thresh"));
  for (const auto & item : planner_only_current.at("values").items()) {
    assert(item.key().rfind("planner.", 0) == 0);
  }

  const auto planner_only_unknown = planner_only.apply("planner.max_yaw_acc", 50.0);
  assert(!planner_only_unknown.ok);
  assert(planner_only_unknown.status == tools::dashboard::DashboardParamStatus::UnknownKey);

  const auto command_payload =
    tools::dashboard::make_control_cmd_payload("request-1", "republish_params", 3456);
  assert(command_payload.contains("command"));
  assert(!command_payload.contains("cmd"));
  assert(command_payload.at("args").is_object());
  assert(command_payload.at("args").empty());

  const auto command_without_args = nlohmann::json{
    {"request_id", "request-2"}, {"command", "stop_dashboard"}, {"timestamp", 4567}};
  assert(tools::dashboard::validate_control_cmd_payload(command_without_args));

  return 0;
}
