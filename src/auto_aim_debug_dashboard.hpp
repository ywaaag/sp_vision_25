#ifndef SRC__AUTO_AIM_DEBUG_DASHBOARD_HPP
#define SRC__AUTO_AIM_DEBUG_DASHBOARD_HPP

#include <memory>

#include <nlohmann/json.hpp>

namespace auto_aim
{
class Planner;
}  // namespace auto_aim

namespace tools
{
namespace dashboard
{
struct DashboardConfig;
}  // namespace dashboard
}  // namespace tools

class AutoAimDebugDashboard
{
public:
  AutoAimDebugDashboard(
    const tools::dashboard::DashboardConfig & config, auto_aim::Planner & planner);
  ~AutoAimDebugDashboard();

  AutoAimDebugDashboard(const AutoAimDebugDashboard &) = delete;
  AutoAimDebugDashboard & operator=(const AutoAimDebugDashboard &) = delete;

  bool enabled() const;
  void handle_commands();
  void push_data(const nlohmann::json & values);
  void stop();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

#endif  // SRC__AUTO_AIM_DEBUG_DASHBOARD_HPP
