#ifndef TOOLS__DASHBOARD_CLI_HPP
#define TOOLS__DASHBOARD_CLI_HPP

#include <optional>
#include <string>
#include <vector>

#include "tools/dashboard_config.hpp"

namespace tools::dashboard::cli
{

std::vector<std::string> normalize_cli_args(int argc, char * argv[]);

std::vector<char *> make_cli_argv(std::vector<std::string> & args);

std::optional<std::string> cli_option_value(
  const std::vector<std::string> & args, const std::string & option);

DashboardConfigOverrides make_dashboard_overrides(
  const std::vector<std::string> & args, bool force_enabled);

}  // namespace tools::dashboard::cli

#endif  // TOOLS__DASHBOARD_CLI_HPP
