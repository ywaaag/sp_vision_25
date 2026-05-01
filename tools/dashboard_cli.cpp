#include "tools/dashboard_cli.hpp"

namespace tools::dashboard::cli
{

std::vector<std::string> normalize_cli_args(int argc, char * argv[])
{
  std::vector<std::string> normalized;
  normalized.reserve(argc);
  for (int i = 0; i < argc; ++i) {
    const std::string arg = argv[i];
    if ((arg == "--robot-id" || arg == "--mqtt-host") && i + 1 < argc) {
      normalized.push_back(arg + "=" + argv[++i]);
    } else {
      normalized.push_back(arg);
    }
  }
  return normalized;
}

std::vector<char *> make_cli_argv(std::vector<std::string> & args)
{
  std::vector<char *> argv;
  argv.reserve(args.size());
  for (auto & arg : args) {
    argv.push_back(arg.data());
  }
  return argv;
}

std::optional<std::string> cli_option_value(
  const std::vector<std::string> & args, const std::string & option)
{
  const auto prefix = option + "=";
  for (const auto & arg : args) {
    if (arg.rfind(prefix, 0) == 0) {
      return arg.substr(prefix.size());
    }
  }
  return std::nullopt;
}

DashboardConfigOverrides make_dashboard_overrides(
  const std::vector<std::string> & args, bool force_enabled)
{
  DashboardConfigOverrides overrides;
  overrides.force_enabled = force_enabled;
  overrides.robot_id = cli_option_value(args, "--robot-id");
  overrides.mqtt_host = cli_option_value(args, "--mqtt-host");
  return overrides;
}

}  // namespace tools::dashboard::cli
