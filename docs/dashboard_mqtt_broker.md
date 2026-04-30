# MQTT Dashboard Broker Runbook

本仓库不再提供 Dashboard broker 或静态页面服务。Mosquitto、WebSocket MQTT 和 HTTP Dashboard 均由独立 Dashboard panel 仓库负责启动。

## Ports Expected By The Vision App

独立 Dashboard 服务应提供：

| Port | Purpose |
| :--- | :--- |
| `1883/tcp` | native MQTT for C++ vision client |
| `9001/tcp` | MQTT over WebSocket for browser client |
| `8080/tcp` | browser Dashboard HTTP |

## Topology A: Same Robot Or Host

Dashboard 服务和 `auto_aim_debug_mpc` 在同一台机器人或主机上：

```text
auto_aim_debug_mpc -> tcp://127.0.0.1:1883
browser device -> http://robot-lan-ip:8080
browser MQTT WS -> ws://robot-lan-ip:9001
```

C++ 使用 `127.0.0.1` 是因为 broker 与视觉程序在同一网络命名空间。浏览器设备必须访问机器人 LAN IP，不能访问浏览器设备自己的 loopback 地址。

## Topology B: Dashboard On Debug PC

Dashboard 服务在调试电脑上，视觉程序在机器人上：

```text
auto_aim_debug_mpc -> tcp://debug-pc-lan-ip:1883
browser device -> http://debug-pc-lan-ip:8080
browser MQTT WS -> ws://debug-pc-lan-ip:9001
```

这种部署下，`dashboard.mqtt_host` 或 `--mqtt-host` 必须使用调试电脑 LAN IP。

## Boundary

`sp_vision_25` 只定义 MQTT 契约和 C++ client，不启动 broker，不提供 WebSocket 服务，不提供 HTTP 静态服务。
