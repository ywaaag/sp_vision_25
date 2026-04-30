#ifndef TOOLS__MQTT_BRIDGE_HPP
#define TOOLS__MQTT_BRIDGE_HPP

#include <mqtt/async_client.h>

#include <atomic>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>

#include "tools/thread_safe_queue.hpp"

namespace tools {

struct MqttBridgeOptions {
  std::string server_uri{"tcp://localhost:1883"};
  std::string client_id{"sp_vision_25"};
  std::string robot_id{"myrobot"};
  std::string username;
  std::string password;
  std::size_t telemetry_queue_size{64};
  std::size_t publish_queue_size{64};
  std::size_t inbound_queue_size{32};
};

enum class MqttCommandType {
  Param,
  Cmd,
};

struct MqttCommand {
  MqttCommandType type{MqttCommandType::Cmd};
  std::string request_id;
  std::string key;
  nlohmann::json value;
  std::string command;
  nlohmann::json args;
  std::int64_t timestamp{0};
  nlohmann::json raw;
};

class MqttBridge : public virtual mqtt::callback {
 public:
  explicit MqttBridge(const MqttBridgeOptions &options);
  ~MqttBridge() override;

  MqttBridge(const MqttBridge &) = delete;
  MqttBridge &operator=(const MqttBridge &) = delete;

  bool start();
  void stop();

  bool push_data(const nlohmann::json &values);
  bool push_data(const nlohmann::json &values, std::int64_t timestamp);
  bool push_log(const std::string &level, const std::string &source,
                const std::string &message);
  bool push_log(const std::string &level, const std::string &source,
                const std::string &message, std::int64_t timestamp);
  bool publish_params_schema(const nlohmann::json &params, int version = 1);
  bool publish_params_schema_payload(const nlohmann::json &payload);
  bool publish_params_current(const nlohmann::json &values);
  bool publish_params_current(const nlohmann::json &values,
                              std::int64_t timestamp);
  bool publish_params_current_payload(const nlohmann::json &payload);
  bool publish_ack(const std::string &request_id, bool ok,
                   const std::string &message, const nlohmann::json &applied);
  bool publish_ack(const std::string &request_id, bool ok,
                   const std::string &message, const nlohmann::json &applied,
                   std::int64_t timestamp);
  bool try_pop_command(MqttCommand &command);

  bool running() const;
  const MqttBridgeOptions &options() const;

 private:
  struct PublishMessage {
    std::string topic;
    std::string payload;
    int qos{0};
  };

  void message_arrived(mqtt::const_message_ptr message) override;
  void publish_loop();
  bool enqueue_publish(const std::string &topic, const nlohmann::json &payload,
                       int qos);
  bool enqueue_telemetry(const nlohmann::json &payload);
  void handle_control_param(const nlohmann::json &payload);
  void handle_control_cmd(const nlohmann::json &payload);

  MqttBridgeOptions options_;
  mqtt::async_client client_;
  mqtt::connect_options connect_options_;
  std::atomic<bool> running_{false};
  std::thread publish_thread_;
  ThreadSafeQueue<PublishMessage, true> telemetry_queue_;
  ThreadSafeQueue<PublishMessage> publish_queue_;
  ThreadSafeQueue<MqttCommand, true> inbound_queue_;
};

std::int64_t dashboard_unix_timestamp_ms();

}  // namespace tools

#endif  // TOOLS__MQTT_BRIDGE_HPP
