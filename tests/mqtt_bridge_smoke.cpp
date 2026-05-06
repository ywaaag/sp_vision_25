#include <chrono>
#include <cstdlib>
#include <exception>
#include <string>
#include <thread>

#include "tools/dashboard_mqtt_contract.hpp"
#include "tools/logger.hpp"
#include "tools/mqtt_bridge.hpp"

namespace {
std::string env_or_default(const char* name, const std::string& fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || std::string(value).empty()) {
    return fallback;
  }
  return value;
}
}  // namespace

int main() {
  tools::MqttBridgeOptions options;
  options.server_uri =
      env_or_default("MQTT_SERVER_URI", "tcp://localhost:1883");
  options.client_id =
      env_or_default("MQTT_CLIENT_ID", "sp_vision_25_mqtt_bridge_smoke");
  options.robot_id = env_or_default("MQTT_ROBOT_ID", "myrobot");
  options.telemetry_queue_size = 4;
  options.publish_queue_size = 8;
  options.inbound_queue_size = 8;

  try {
    tools::MqttBridge bridge(options);
    bridge.start();

    const auto timestamp = tools::dashboard_unix_timestamp_ms();
    bridge.push_data(nlohmann::json{{"fps", 120.0}, {"target_found", true}},
                     timestamp);
    bridge.push_log("info", "mqtt_bridge_smoke", "bridge smoke message",
                    timestamp);
    bridge.publish_params_schema_payload(tools::dashboard::make_params_schema_payload(
        {tools::dashboard::make_number_param_schema(
            "camera.exposure", 5000.0, 0.0, 20000.0, 100.0, "us",
            "camera")}));
    bridge.publish_params_current(nlohmann::json{{"camera.exposure", 5000}},
                                  timestamp);
    bridge.publish_ack("smoke-request", true, "smoke ack",
                       nlohmann::json::object(), timestamp);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    bridge.stop();
  } catch (const std::exception& error) {
    tools::logger()->error("mqtt_bridge_smoke failed: {}", error.what());
    return 1;
  }

  return 0;
}
