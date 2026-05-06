#ifndef TOOLS__DASHBOARD_PARAMS_HPP
#define TOOLS__DASHBOARD_PARAMS_HPP

#include <functional>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace auto_aim
{
class Planner;
}  // namespace auto_aim

namespace tools
{
namespace dashboard
{

struct DashboardParamSnapshot
{
  double planner_yaw_offset_deg;
  double planner_pitch_offset_deg;
  double planner_fire_thresh;
  double planner_decision_speed;
  double planner_high_speed_delay_time;
  double planner_low_speed_delay_time;
};

struct DashboardParamUpdate
{
  std::string key;
  std::string local_key;
  double value;
};

enum class DashboardParamStatus
{
  Applied,
  UnknownKey,
  TypeError,
  OutOfRange,
  ApplyFailed
};

struct DashboardParamResult
{
  bool ok;
  DashboardParamStatus status;
  std::string key;
  std::string message;
  nlohmann::json applied;
};

class DashboardParams
{
public:
  using SnapshotReader = std::function<DashboardParamSnapshot()>;
  using ParamWriter = std::function<bool(const DashboardParamUpdate &)>;

  DashboardParams(SnapshotReader read_snapshot, ParamWriter write_param, bool include_buff = true);
  explicit DashboardParams(auto_aim::Planner & planner);
  DashboardParams(
    const std::string & config_path, auto_aim::Planner & planner, bool include_buff = false);

  nlohmann::json make_schema() const;
  nlohmann::json make_current(std::int64_t timestamp) const;
  DashboardParamResult apply(const std::string & key, const nlohmann::json & value) const;

public:
  struct ParamSpec
  {
    std::string key;
    std::string local_key;
    std::string group;
    std::string type;
    nlohmann::json value;
    double DashboardParamSnapshot::*snapshot_value{nullptr};
    bool editable{true};
    bool restart_required{false};
    bool render_json{false};
    bool has_min{false};
    bool has_max{false};
    bool has_step{false};
    double min{0.0};
    double max{0.0};
    double step{0.0};
    std::string unit;
    std::vector<std::string> options;
    std::string source_key;
  };

private:
  SnapshotReader read_snapshot_;
  ParamWriter write_param_;
  bool include_buff_{true};
  std::vector<ParamSpec> params_;
};

}  // namespace dashboard
}  // namespace tools

#endif  // TOOLS__DASHBOARD_PARAMS_HPP
