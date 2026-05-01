#ifndef TOOLS__DASHBOARD_MQTT_CONTRACT_HPP
#define TOOLS__DASHBOARD_MQTT_CONTRACT_HPP

#include <algorithm>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace tools
{
namespace dashboard
{

inline constexpr char TOPIC_DATA_SUFFIX[] = "data";
inline constexpr char TOPIC_LOG_SUFFIX[] = "log";
inline constexpr char TOPIC_PARAMS_SCHEMA_SUFFIX[] = "params/schema";
inline constexpr char TOPIC_PARAMS_CURRENT_SUFFIX[] = "params/current";
inline constexpr char TOPIC_CONTROL_PARAM_SUFFIX[] = "control/param";
inline constexpr char TOPIC_CONTROL_CMD_SUFFIX[] = "control/cmd";
inline constexpr char TOPIC_CONTROL_ACK_SUFFIX[] = "control/ack";

inline constexpr int QOS_DATA = 0;
inline constexpr int QOS_LOG = 0;
inline constexpr int QOS_PARAMS_SCHEMA = 0;
inline constexpr int QOS_PARAMS_CURRENT = 0;
inline constexpr int QOS_CONTROL_PARAM = 1;
inline constexpr int QOS_CONTROL_CMD = 1;
inline constexpr int QOS_CONTROL_ACK = 1;

inline bool is_valid_robot_id(const std::string & robot_id)
{
  return !robot_id.empty() && robot_id.find('/') == std::string::npos &&
         robot_id.find('+') == std::string::npos && robot_id.find('#') == std::string::npos;
}

inline std::string make_topic(const std::string & robot_id, const std::string & suffix)
{
  return robot_id + "/" + suffix;
}

inline std::string data_topic(const std::string & robot_id)
{
  return make_topic(robot_id, TOPIC_DATA_SUFFIX);
}

inline std::string log_topic(const std::string & robot_id)
{
  return make_topic(robot_id, TOPIC_LOG_SUFFIX);
}

inline std::string params_schema_topic(const std::string & robot_id)
{
  return make_topic(robot_id, TOPIC_PARAMS_SCHEMA_SUFFIX);
}

inline std::string params_current_topic(const std::string & robot_id)
{
  return make_topic(robot_id, TOPIC_PARAMS_CURRENT_SUFFIX);
}

inline std::string control_param_topic(const std::string & robot_id)
{
  return make_topic(robot_id, TOPIC_CONTROL_PARAM_SUFFIX);
}

inline std::string control_cmd_topic(const std::string & robot_id)
{
  return make_topic(robot_id, TOPIC_CONTROL_CMD_SUFFIX);
}

inline std::string control_ack_topic(const std::string & robot_id)
{
  return make_topic(robot_id, TOPIC_CONTROL_ACK_SUFFIX);
}

inline int qos_for_topic_suffix(const std::string & suffix)
{
  if (suffix == TOPIC_CONTROL_PARAM_SUFFIX) {
    return QOS_CONTROL_PARAM;
  }
  if (suffix == TOPIC_CONTROL_CMD_SUFFIX) {
    return QOS_CONTROL_CMD;
  }
  if (suffix == TOPIC_CONTROL_ACK_SUFFIX) {
    return QOS_CONTROL_ACK;
  }
  if (suffix == TOPIC_LOG_SUFFIX) {
    return QOS_LOG;
  }
  if (suffix == TOPIC_PARAMS_SCHEMA_SUFFIX) {
    return QOS_PARAMS_SCHEMA;
  }
  if (suffix == TOPIC_PARAMS_CURRENT_SUFFIX) {
    return QOS_PARAMS_CURRENT;
  }
  if (suffix == TOPIC_DATA_SUFFIX) {
    return QOS_DATA;
  }
  return -1;
}

inline nlohmann::json make_data_payload(const nlohmann::json & values, std::int64_t timestamp)
{
  return nlohmann::json{{"timestamp", timestamp}, {"values", values}};
}

inline nlohmann::json make_log_payload(
  const std::string & level, const std::string & source, const std::string & message,
  std::int64_t timestamp)
{
  return nlohmann::json{
    {"timestamp", timestamp}, {"level", level}, {"source", source}, {"message", message}};
}

inline nlohmann::json make_number_param_schema(
  const std::string & key, double value, double min, double max, double step,
  const std::string & unit, const std::string & group)
{
  return nlohmann::json{{"key", key},   {"type", "number"}, {"min", min},     {"max", max},
                        {"step", step}, {"unit", unit},     {"group", group}, {"value", value}};
}

inline nlohmann::json make_bool_param_schema(
  const std::string & key, bool value, const std::string & group)
{
  return nlohmann::json{{"key", key}, {"type", "bool"}, {"group", group}, {"value", value}};
}

inline nlohmann::json make_enum_param_schema(
  const std::string & key, const std::string & value, const std::vector<std::string> & options,
  const std::string & group)
{
  return nlohmann::json{
    {"key", key}, {"type", "enum"}, {"group", group}, {"value", value}, {"options", options}};
}

inline nlohmann::json make_string_param_schema(
  const std::string & key, const std::string & value, const std::string & group,
  const std::string & unit = "")
{
  auto payload =
    nlohmann::json{{"key", key}, {"type", "string"}, {"group", group}, {"value", value}};
  if (!unit.empty()) {
    payload["unit"] = unit;
  }
  return payload;
}

inline nlohmann::json make_params_schema_payload(
  const std::vector<nlohmann::json> & params, int version = 1)
{
  return nlohmann::json{{"version", version}, {"params", params}};
}

inline nlohmann::json make_params_current_payload(
  const nlohmann::json & values, std::int64_t timestamp)
{
  return nlohmann::json{{"timestamp", timestamp}, {"values", values}};
}

inline nlohmann::json make_control_param_payload(
  const std::string & request_id, const std::string & key, const nlohmann::json & value,
  std::int64_t timestamp)
{
  return nlohmann::json{
    {"request_id", request_id}, {"key", key}, {"value", value}, {"timestamp", timestamp}};
}

inline nlohmann::json make_control_cmd_payload(
  const std::string & request_id, const std::string & command, const nlohmann::json & args,
  std::int64_t timestamp)
{
  return nlohmann::json{
    {"request_id", request_id}, {"command", command}, {"args", args}, {"timestamp", timestamp}};
}

inline nlohmann::json make_control_cmd_payload(
  const std::string & request_id, const std::string & command, std::int64_t timestamp)
{
  return make_control_cmd_payload(request_id, command, nlohmann::json::object(), timestamp);
}

inline nlohmann::json make_control_ack_payload(
  const std::string & request_id, bool ok, const std::string & message,
  const nlohmann::json & applied, std::int64_t timestamp)
{
  return nlohmann::json{
    {"request_id", request_id},
    {"ok", ok},
    {"message", message},
    {"applied", applied},
    {"timestamp", timestamp}};
}

namespace detail
{
inline bool has_string(const nlohmann::json & payload, const char * key)
{
  return payload.contains(key) && payload.at(key).is_string();
}

inline bool has_integer(const nlohmann::json & payload, const char * key)
{
  return payload.contains(key) && payload.at(key).is_number_integer();
}

inline bool has_object(const nlohmann::json & payload, const char * key)
{
  return payload.contains(key) && payload.at(key).is_object();
}

inline bool has_value(const nlohmann::json & payload, const char * key)
{
  return payload.contains(key) && !payload.at(key).is_null();
}

inline bool contains_string_value(const nlohmann::json & values, const std::string & value)
{
  return std::any_of(values.begin(), values.end(), [&value](const auto & item) {
    return item.is_string() && item.template get<std::string>() == value;
  });
}

inline bool all_values_are_strings(const nlohmann::json & values)
{
  return std::all_of(
    values.begin(), values.end(), [](const auto & item) { return item.is_string(); });
}
}  // namespace detail

inline bool validate_data_payload(const nlohmann::json & payload)
{
  return payload.is_object() && detail::has_integer(payload, "timestamp") &&
         detail::has_object(payload, "values");
}

inline bool validate_log_payload(const nlohmann::json & payload)
{
  return payload.is_object() && detail::has_integer(payload, "timestamp") &&
         detail::has_string(payload, "level") && detail::has_string(payload, "source") &&
         detail::has_string(payload, "message");
}

inline bool validate_param_schema_entry(const nlohmann::json & entry)
{
  if (
    !entry.is_object() || !detail::has_string(entry, "key") || !detail::has_string(entry, "type") ||
    !detail::has_string(entry, "group") || !detail::has_value(entry, "value")) {
    return false;
  }

  if (entry.contains("unit") && !entry.at("unit").is_string()) {
    return false;
  }

  const auto type = entry.at("type").get<std::string>();
  if (type == "number") {
    if (!entry.at("value").is_number()) {
      return false;
    }
    for (const auto * key : {"min", "max", "step"}) {
      if (entry.contains(key) && !entry.at(key).is_number()) {
        return false;
      }
    }
    return !entry.contains("options");
  }

  if (entry.contains("min") || entry.contains("max") || entry.contains("step")) {
    return false;
  }

  if (type == "bool") {
    return entry.at("value").is_boolean() && !entry.contains("options");
  }

  if (type == "enum") {
    if (
      !entry.at("value").is_string() || !entry.contains("options") ||
      !entry.at("options").is_array() || !detail::all_values_are_strings(entry.at("options"))) {
      return false;
    }
    return detail::contains_string_value(entry.at("options"), entry.at("value").get<std::string>());
  }

  if (type == "string") {
    return entry.at("value").is_string() && !entry.contains("options");
  }

  return false;
}

inline bool validate_params_schema_payload(const nlohmann::json & payload)
{
  if (
    !payload.is_object() || !payload.contains("version") ||
    !payload.at("version").is_number_integer() || !payload.contains("params") ||
    !payload.at("params").is_array()) {
    return false;
  }

  return std::all_of(
    payload.at("params").begin(), payload.at("params").end(),
    [](const auto & item) { return validate_param_schema_entry(item); });
}

inline bool validate_params_current_payload(const nlohmann::json & payload)
{
  return payload.is_object() && detail::has_integer(payload, "timestamp") &&
         detail::has_object(payload, "values");
}

inline bool validate_control_param_payload(const nlohmann::json & payload)
{
  return payload.is_object() && detail::has_string(payload, "request_id") &&
         detail::has_string(payload, "key") && detail::has_value(payload, "value") &&
         detail::has_integer(payload, "timestamp");
}

inline bool validate_control_cmd_payload(const nlohmann::json & payload)
{
  return payload.is_object() && detail::has_string(payload, "request_id") &&
         detail::has_string(payload, "command") &&
         (!payload.contains("args") || detail::has_object(payload, "args")) &&
         detail::has_integer(payload, "timestamp");
}

inline bool validate_control_ack_payload(const nlohmann::json & payload)
{
  return payload.is_object() && detail::has_string(payload, "request_id") &&
         payload.contains("ok") && payload.at("ok").is_boolean() &&
         detail::has_string(payload, "message") && detail::has_object(payload, "applied") &&
         detail::has_integer(payload, "timestamp");
}

}  // namespace dashboard
}  // namespace tools

#endif  // TOOLS__DASHBOARD_MQTT_CONTRACT_HPP
