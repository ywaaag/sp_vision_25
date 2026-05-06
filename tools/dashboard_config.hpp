#ifndef TOOLS__DASHBOARD_CONFIG_HPP
#define TOOLS__DASHBOARD_CONFIG_HPP

#include <optional>
#include <string>

namespace tools
{
namespace dashboard
{

struct DashboardConfig
{
  bool enabled{false};
  std::string robot_id{"myrobot"};
  std::string mqtt_host{"tcp://127.0.0.1:1883"};
};

struct DashboardConfigOverrides
{
  bool force_enabled{false};
  std::optional<std::string> robot_id;
  std::optional<std::string> mqtt_host;
};

DashboardConfig load_dashboard_config(
  const std::string & config_path, const DashboardConfigOverrides & overrides = {});

}  // namespace dashboard
}  // namespace tools

#endif  // TOOLS__DASHBOARD_CONFIG_HPP
