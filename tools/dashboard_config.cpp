#include "tools/dashboard_config.hpp"

#include <yaml-cpp/yaml.h>

namespace tools
{
namespace dashboard
{
namespace
{

std::string non_empty_or_default(const YAML::Node & node, const std::string & default_value)
{
  if (!node) {
    return default_value;
  }
  const auto value = node.as<std::string>();
  return value.empty() ? default_value : value;
}

}  // namespace

DashboardConfig load_dashboard_config(
  const std::string & config_path, const DashboardConfigOverrides & overrides)
{
  DashboardConfig config;

  const auto yaml = YAML::LoadFile(config_path);
  const auto dashboard = yaml["dashboard"];
  if (dashboard) {
    if (dashboard["enabled"]) {
      config.enabled = dashboard["enabled"].as<bool>();
    }
    config.robot_id = non_empty_or_default(dashboard["robot_id"], config.robot_id);
    config.mqtt_host = non_empty_or_default(dashboard["mqtt_host"], config.mqtt_host);
  }

  if (overrides.force_enabled) {
    config.enabled = true;
  }
  if (overrides.robot_id.has_value() && !overrides.robot_id->empty()) {
    config.robot_id = *overrides.robot_id;
  }
  if (overrides.mqtt_host.has_value() && !overrides.mqtt_host->empty()) {
    config.mqtt_host = *overrides.mqtt_host;
  }

  return config;
}

}  // namespace dashboard
}  // namespace tools
