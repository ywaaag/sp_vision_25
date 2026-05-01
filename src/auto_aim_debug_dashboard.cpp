#include "src/auto_aim_debug_dashboard.hpp"

#include "tools/dashboard_config.hpp"
#include "tools/logger.hpp"

#ifdef SP_VISION_ENABLE_DASHBOARD_MQTT
#include <atomic>
#include <exception>
#include <memory>
#include <utility>

#include "tools/dashboard_params.hpp"
#include "tools/mqtt_bridge.hpp"
#endif

class AutoAimDebugDashboard::Impl
{
public:
  Impl(
    const tools::dashboard::DashboardConfig & config, const std::string & config_path,
    auto_aim::Planner & planner)
#ifdef SP_VISION_ENABLE_DASHBOARD_MQTT
  : telemetry_enabled_(config.enabled)
#endif
  {
#ifdef SP_VISION_ENABLE_DASHBOARD_MQTT
    if (!config.enabled) {
      return;
    }

    try {
      tools::MqttBridgeOptions options;
      options.server_uri = config.mqtt_host;
      options.robot_id = config.robot_id;
      options.client_id = options.robot_id + "_auto_aim_debug_mpc";

      auto next_params =
        std::make_unique<tools::dashboard::DashboardParams>(config_path, planner, false);
      auto next_bridge = std::make_unique<tools::MqttBridge>(options);
      next_bridge->start();
      publish_params(*next_bridge, *next_params);
      bridge_ = std::move(next_bridge);
      params_ = std::move(next_params);
      tools::logger()->info(
        "MQTT Dashboard enabled for {} at {}", options.robot_id, options.server_uri);
    } catch (const std::exception & e) {
      telemetry_enabled_.store(false);
      tools::logger()->warn("MQTT Dashboard disabled: {}", e.what());
    } catch (...) {
      telemetry_enabled_.store(false);
      tools::logger()->warn("MQTT Dashboard disabled: unknown initialization error");
    }
#else
    (void)config_path;
    (void)planner;
    if (config.enabled) {
      tools::logger()->warn("MQTT Dashboard requested but mqtt_bridge was not built");
    }
#endif
  }

  ~Impl() { stop(); }

  bool enabled() const
  {
#ifdef SP_VISION_ENABLE_DASHBOARD_MQTT
    return bridge_ != nullptr;
#else
    return false;
#endif
  }

  void handle_commands()
  {
#ifdef SP_VISION_ENABLE_DASHBOARD_MQTT
    if (!bridge_ || !params_) {
      return;
    }

    tools::MqttCommand command;
    while (bridge_->try_pop_command(command)) {
      if (command.type == tools::MqttCommandType::Param) {
        const auto result = params_->apply(command.key, command.value);
        if (result.ok) {
          bridge_->publish_params_current_payload(
            params_->make_current(tools::dashboard_unix_timestamp_ms()));
        }
        bridge_->publish_ack(command.request_id, result.ok, result.message, result.applied);
        continue;
      }

      if (command.command == "stop_dashboard") {
        telemetry_enabled_.store(false);
        bridge_->publish_ack(
          command.request_id, true, "dashboard telemetry stopped",
          nlohmann::json{{"command", command.command}});
      } else if (command.command == "start_dashboard") {
        telemetry_enabled_.store(true);
        bridge_->publish_ack(
          command.request_id, true, "dashboard telemetry started",
          nlohmann::json{{"command", command.command}});
      } else if (command.command == "republish_params") {
        publish_params();
        bridge_->publish_ack(
          command.request_id, true, "dashboard parameters republished",
          nlohmann::json{{"command", command.command}});
      } else {
        bridge_->publish_ack(
          command.request_id, false, "unknown dashboard command", nlohmann::json::object());
      }
    }
#endif
  }

  void push_data(const nlohmann::json & values)
  {
#ifdef SP_VISION_ENABLE_DASHBOARD_MQTT
    if (bridge_ && telemetry_enabled_.load()) {
      bridge_->push_data(values);
    }
#else
    (void)values;
#endif
  }

  void stop()
  {
#ifdef SP_VISION_ENABLE_DASHBOARD_MQTT
    if (bridge_) {
      bridge_->stop();
    }
#endif
  }

private:
#ifdef SP_VISION_ENABLE_DASHBOARD_MQTT
  void publish_params()
  {
    if (!bridge_ || !params_) {
      return;
    }

    publish_params(*bridge_, *params_);
  }

  void publish_params(
    tools::MqttBridge & bridge, const tools::dashboard::DashboardParams & params)
  {
    const auto timestamp = tools::dashboard_unix_timestamp_ms();
    bridge.publish_params_schema_payload(params.make_schema());
    bridge.publish_params_current_payload(params.make_current(timestamp));
  }

  std::unique_ptr<tools::MqttBridge> bridge_;
  std::unique_ptr<tools::dashboard::DashboardParams> params_;
  std::atomic<bool> telemetry_enabled_{false};
#endif
};

AutoAimDebugDashboard::AutoAimDebugDashboard(
  const tools::dashboard::DashboardConfig & config, const std::string & config_path,
  auto_aim::Planner & planner)
: impl_(std::make_unique<Impl>(config, config_path, planner))
{
}

AutoAimDebugDashboard::~AutoAimDebugDashboard() = default;

bool AutoAimDebugDashboard::enabled() const
{
  return impl_->enabled();
}

void AutoAimDebugDashboard::handle_commands()
{
  impl_->handle_commands();
}

void AutoAimDebugDashboard::push_data(const nlohmann::json & values)
{
  impl_->push_data(values);
}

void AutoAimDebugDashboard::stop()
{
  impl_->stop();
}
