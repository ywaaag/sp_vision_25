# Dashboard Frontend Split

领导要求 `sp_vision_25` 长期保持为视觉代码仓库，因此 Dashboard panel 前端已从本仓库拆出。当前仓库只保留 MQTT 后端能力：topic/payload 契约、C++ bridge、参数 schema/current 和 `auto_aim_debug_mpc` 接入。

## Frontend Repository

正式前端仓库路径：

```text
/home/ywag/sp_vision_dashboard_panel
```

该仓库职责：

- 提供 Dashboard browser UI。
- 提供 style assets 和 browser-side modules。
- 提供 Mosquitto native MQTT `1883`。
- 提供 MQTT over WebSocket `9001`。
- 提供 HTTP Dashboard `8080`。
- 不包含视觉算法代码。
- 只通过 MQTT 协议和 `sp_vision_25` 通信。

## Current Split

`sp_vision_25` 只维护：

- MQTT topic/payload 契约。
- C++ `MqttBridge`。
- `DashboardParams`。
- `auto_aim_debug_mpc` 接入。
- Paho MQTT C++ 可选探测和链接逻辑。

`sp_vision_dashboard_panel` 维护：

- Dashboard UI。
- CSS/JS/vendor。
- Mosquitto container。
- HTTP static server。
- Dashboard startup/check/smoke scripts。

`standard_mpc.cpp` 不接入 Dashboard，保持 upstream/main 行为。

## Frontend Repository Layout

```text
sp_vision_dashboard_panel/
  dashboard/
    index.html
    css/
    js/
    vendor/
  docker/
    Dockerfile
    mosquitto.conf
    entrypoint.sh
  docker-compose.yml
  scripts/
    dashboard_up.sh
    dashboard_down.sh
    dashboard_check.sh
    dashboard_smoke.sh
  docs/
    usage.md
    protocol.md
  README.md
```

## Seed Copy Status

`/tmp/sp_vision_dashboard_panel_seed/` 只是本次迁移使用的临时种子目录，不是长期方案。正式前端成果已经落到：

```text
/home/ywag/sp_vision_dashboard_panel
```

## Contract Source

协议仍以本仓库为准：

```text
docs/dashboard_mqtt_protocol.md
tools/dashboard_mqtt_contract.hpp
```

前端仓库应复制或引用协议文档，但不能绕过该契约直接改变 topic、payload、QoS 或控制语义。

## Deployment Reminder

同机部署：

```text
auto_aim_debug_mpc -> tcp://127.0.0.1:1883
browser device -> http://robot-lan-ip:8080
browser MQTT WS -> ws://robot-lan-ip:9001
```

分离部署：

```text
auto_aim_debug_mpc -> tcp://debug-pc-lan-ip:1883
browser device -> http://debug-pc-lan-ip:8080
browser MQTT WS -> ws://debug-pc-lan-ip:9001
```
