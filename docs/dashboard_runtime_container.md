# Dashboard Runtime Boundary

本仓库只保留视觉程序侧的 MQTT Dashboard 后端能力，不再承载浏览器 UI、静态 HTTP 服务或前端容器。

## Repository Split

- `sp_vision_25`: 视觉代码仓库，负责 MQTT topic、payload、C++ bridge、参数 schema/current 和控制消息消费。
- 独立 Dashboard panel 仓库：负责浏览器页面、样式、前端脚本、第三方前端依赖、Mosquitto、WebSocket MQTT 和 HTTP 静态服务。

两边只通过 `docs/dashboard_mqtt_protocol.md` 定义的 MQTT 契约通信。

## Vision App

Dashboard 当前只接入 `auto_aim_debug_mpc`。`standard_mpc` 保持 upstream/main 行为，不作为 Dashboard 入口。

```bash
./build/auto_aim_debug_mpc --dashboard configs/standard3.yaml
```

也可以显式覆盖 broker 地址：

```bash
./build/auto_aim_debug_mpc --dashboard --mqtt-host tcp://127.0.0.1:1883 configs/standard3.yaml
```

YAML 配置段：

```yaml
dashboard:
  enabled: false
  robot_id: "myrobot"
  mqtt_host: "tcp://127.0.0.1:1883"
```

部署时也可以改为：

```yaml
dashboard:
  enabled: true
  robot_id: "myrobot"
  mqtt_host: "tcp://127.0.0.1:1883"
```

CLI 覆盖优先级高于 YAML。MQTT 初始化或连接失败时，视觉程序记录 warning，并降级关闭 Dashboard telemetry，不直接退出主视觉业务。

## Network Topologies

拓扑 A：Dashboard 服务和视觉程序在同一台机器人或主机上。

```text
auto_aim_debug_mpc -> tcp://127.0.0.1:1883
browser device -> http://robot-lan-ip:8080
browser MQTT WS -> ws://robot-lan-ip:9001
```

这里 C++ 用 `127.0.0.1` 是合理的，因为 broker 在视觉程序同一主机网络命名空间。电脑或平板浏览器不能访问自己的 `127.0.0.1:8080`，必须访问机器人 LAN IP。

拓扑 B：Dashboard 服务在调试电脑上，视觉程序在机器人上。

```text
auto_aim_debug_mpc -> tcp://debug-pc-lan-ip:1883
browser device -> http://debug-pc-lan-ip:8080
browser MQTT WS -> ws://debug-pc-lan-ip:9001
```

如果 broker 不在视觉程序同一网络命名空间，`dashboard.mqtt_host` 或 `--mqtt-host` 必须使用 Dashboard 主机 LAN IP。

## Frontend Runtime

Dashboard 服务由独立前端仓库启动：

```bash
docker compose up -d
```

该仓库应提供：

- Mosquitto native MQTT: `1883`
- MQTT over WebSocket: `9001`
- HTTP static Dashboard: `8080`

`sp_vision_25` 中的 C++ 代码不提供 HTML、CSS、JS 或 HTTP static server。

## Production Boundary

本仓库不包含 image/video/MJPEG、Web terminal、mock runtime、video-source、hardwareless smoke 或 mock publisher。视觉程序只作为 MQTT client 使用 native `1883` 与 Dashboard 服务通信。
