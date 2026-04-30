#include "tools/mqtt_bridge.hpp"

#include <chrono>
#include <stdexcept>

#include "tools/dashboard_mqtt_contract.hpp"

namespace tools
{
namespace
{
constexpr auto PUBLISH_IDLE_WAIT = std::chrono::milliseconds(50);

std::int64_t json_timestamp_or_zero(const nlohmann::json &payload) {
  if (payload.contains("timestamp") &&
      payload.at("timestamp").is_number_integer()) {
    return payload.at("timestamp").get<std::int64_t>();
  }
  return 0;
}
}  // namespace

std::int64_t dashboard_unix_timestamp_ms() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
}

MqttBridge::MqttBridge(const MqttBridgeOptions &options)
    : options_(options),
      client_(options_.server_uri, options_.client_id),
      telemetry_queue_(options_.telemetry_queue_size),
      publish_queue_(options_.publish_queue_size),
      inbound_queue_(options_.inbound_queue_size) {
  if (!dashboard::is_valid_robot_id(options_.robot_id)) {
    throw std::invalid_argument(
        "MqttBridge robot_id must be one MQTT topic segment");
  }

  connect_options_.set_clean_session(true);
  if (!options_.username.empty()) {
    connect_options_.set_user_name(options_.username);
    connect_options_.set_password(options_.password);
  }
  client_.set_callback(*this);
}

MqttBridge::~MqttBridge() { stop(); }

bool MqttBridge::start() {
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) {
    return true;
  }

  telemetry_queue_.reopen();
  publish_queue_.reopen();
  inbound_queue_.reopen();

  try {
    client_.connect(connect_options_)->wait();
    client_
        .subscribe(dashboard::control_param_topic(options_.robot_id),
                   dashboard::QOS_CONTROL_PARAM)
        ->wait();
    client_
        .subscribe(dashboard::control_cmd_topic(options_.robot_id),
                   dashboard::QOS_CONTROL_CMD)
        ->wait();
    publish_thread_ = std::thread(&MqttBridge::publish_loop, this);
  } catch (...) {
    running_.store(false);
    telemetry_queue_.close();
    publish_queue_.close();
    inbound_queue_.close();
    throw;
  }

  return true;
}

void MqttBridge::stop() {
  bool expected = true;
  if (!running_.compare_exchange_strong(expected, false)) {
    telemetry_queue_.close();
    publish_queue_.close();
    inbound_queue_.close();
    if (publish_thread_.joinable()) {
      publish_thread_.join();
    }
    return;
  }

  try {
    if (client_.is_connected()) {
      client_.unsubscribe(dashboard::control_param_topic(options_.robot_id))
          ->wait();
      client_.unsubscribe(dashboard::control_cmd_topic(options_.robot_id))
          ->wait();
      client_.disconnect()->wait();
    }
  } catch (const mqtt::exception &) {
  }

  telemetry_queue_.close();
  publish_queue_.close();
  inbound_queue_.close();

  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

bool MqttBridge::push_data(const nlohmann::json &values) {
  return push_data(values, dashboard_unix_timestamp_ms());
}

bool MqttBridge::push_data(const nlohmann::json &values,
                           std::int64_t timestamp) {
  if (!values.is_object()) {
    return false;
  }
  return enqueue_telemetry(dashboard::make_data_payload(values, timestamp));
}

bool MqttBridge::push_log(const std::string &level, const std::string &source,
                          const std::string &message) {
  return push_log(level, source, message, dashboard_unix_timestamp_ms());
}

bool MqttBridge::push_log(const std::string &level, const std::string &source,
                          const std::string &message, std::int64_t timestamp) {
  return enqueue_publish(
      dashboard::log_topic(options_.robot_id),
      dashboard::make_log_payload(level, source, message, timestamp),
      dashboard::QOS_LOG);
}

bool MqttBridge::publish_params_schema(const nlohmann::json &params,
                                       int version) {
  if (!params.is_array()) {
    return false;
  }
  const auto payload = dashboard::make_params_schema_payload(
      params.get<std::vector<nlohmann::json>>(), version);
  return publish_params_schema_payload(payload);
}

bool MqttBridge::publish_params_schema_payload(const nlohmann::json &payload) {
  if (!dashboard::validate_params_schema_payload(payload)) {
    return false;
  }
  return enqueue_publish(dashboard::params_schema_topic(options_.robot_id),
                         payload, dashboard::QOS_PARAMS_SCHEMA);
}

bool MqttBridge::publish_params_current(const nlohmann::json &values) {
  return publish_params_current(values, dashboard_unix_timestamp_ms());
}

bool MqttBridge::publish_params_current(const nlohmann::json &values,
                                        std::int64_t timestamp) {
  if (!values.is_object()) {
    return false;
  }
  return publish_params_current_payload(
      dashboard::make_params_current_payload(values, timestamp));
}

bool MqttBridge::publish_params_current_payload(const nlohmann::json &payload) {
  if (!dashboard::validate_params_current_payload(payload)) {
    return false;
  }
  return enqueue_publish(dashboard::params_current_topic(options_.robot_id),
                         payload, dashboard::QOS_PARAMS_CURRENT);
}

bool MqttBridge::publish_ack(const std::string &request_id, bool ok,
                             const std::string &message,
                             const nlohmann::json &applied) {
  return publish_ack(request_id, ok, message, applied,
                     dashboard_unix_timestamp_ms());
}

bool MqttBridge::publish_ack(const std::string &request_id, bool ok,
                             const std::string &message,
                             const nlohmann::json &applied,
                             std::int64_t timestamp) {
  if (!applied.is_object()) {
    return false;
  }
  return enqueue_publish(dashboard::control_ack_topic(options_.robot_id),
                         dashboard::make_control_ack_payload(
                             request_id, ok, message, applied, timestamp),
                         dashboard::QOS_CONTROL_ACK);
}

bool MqttBridge::try_pop_command(MqttCommand &command) {
  return inbound_queue_.try_pop(command);
}

bool MqttBridge::running() const { return running_.load(); }

const MqttBridgeOptions &MqttBridge::options() const { return options_; }

void MqttBridge::message_arrived(mqtt::const_message_ptr message) {
  if (!message) {
    return;
  }

  nlohmann::json payload;
  try {
    payload = nlohmann::json::parse(message->to_string());
  } catch (const nlohmann::json::exception &) {
    return;
  }

  const auto topic = message->get_topic();
  if (topic == dashboard::control_param_topic(options_.robot_id)) {
    handle_control_param(payload);
    return;
  }
  if (topic == dashboard::control_cmd_topic(options_.robot_id)) {
    handle_control_cmd(payload);
  }
}

void MqttBridge::publish_loop() {
  while (true) {
    PublishMessage message;
    if (publish_queue_.try_pop(message) || telemetry_queue_.try_pop(message) ||
        publish_queue_.wait_pop_for(message, PUBLISH_IDLE_WAIT) ||
        telemetry_queue_.try_pop(message)) {
      try {
        auto mqtt_message = mqtt::make_message(message.topic, message.payload);
        mqtt_message->set_qos(message.qos);
        client_.publish(mqtt_message)->wait();
      } catch (const mqtt::exception &) {
      }
      continue;
    }

    if (!running_.load() && publish_queue_.closed() &&
        telemetry_queue_.closed()) {
      break;
    }
  }
}

bool MqttBridge::enqueue_publish(const std::string &topic,
                                 const nlohmann::json &payload, int qos) {
  if (!running_.load()) {
    return false;
  }

  return publish_queue_.try_push(PublishMessage{topic, payload.dump(), qos});
}

bool MqttBridge::enqueue_telemetry(const nlohmann::json &payload) {
  if (!running_.load()) {
    return false;
  }

  return telemetry_queue_.try_push(
      PublishMessage{dashboard::data_topic(options_.robot_id), payload.dump(),
                     dashboard::QOS_DATA});
}

void MqttBridge::handle_control_param(const nlohmann::json &payload) {
  if (!dashboard::validate_control_param_payload(payload)) {
    return;
  }

  MqttCommand command;
  command.type = MqttCommandType::Param;
  command.request_id = payload.at("request_id").get<std::string>();
  command.key = payload.at("key").get<std::string>();
  command.value = payload.at("value");
  command.timestamp = json_timestamp_or_zero(payload);
  command.raw = payload;
  inbound_queue_.try_push(command);
}

void MqttBridge::handle_control_cmd(const nlohmann::json &payload) {
  if (!dashboard::validate_control_cmd_payload(payload)) {
    return;
  }

  MqttCommand command;
  command.type = MqttCommandType::Cmd;
  command.request_id = payload.at("request_id").get<std::string>();
  command.command = payload.at("command").get<std::string>();
  command.args = payload.contains("args") ? payload.at("args") : nlohmann::json::object();
  command.timestamp = json_timestamp_or_zero(payload);
  command.raw = payload;
  inbound_queue_.try_push(command);
}

}  // namespace tools
