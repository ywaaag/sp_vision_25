# MQTT Dashboard Changelog And Usage

本文面向合并评审和后续使用者，说明本分支在视觉仓库中保留的 MQTT Dashboard 后端能力和生产边界。

## 变更摘要

- 保留 MQTT 通信层：`tools/mqtt_bridge.*`、`tools/dashboard_mqtt_contract.hpp`。
- 保留 Dashboard 参数模型与配置解析：`tools/dashboard_params.*`、`tools/dashboard_config.*`、`tools/dashboard_cli.*`。
- Dashboard 当前只接入 `auto_aim_debug_mpc`：发布 telemetry、log、params/schema、params/current，并消费 control/param、control/cmd。
- `standard_mpc` 保持 upstream/main 行为，不链接 Dashboard 目标。
- `configs/standard3.yaml` 保留 `dashboard` 配置段。
- 浏览器 UI、样式、前端脚本、前端第三方依赖和静态服务容器已从视觉仓库拆出，迁移到独立 Dashboard panel 仓库。

本功能不包含图像流、视频流、MJPEG 或 Web terminal。

## 当前仓库职责

`sp_vision_25` 只负责：

- MQTT topic 与 JSON payload 契约。
- C++ MQTT bridge。
- 参数 schema/current 生成。
- `auto_aim_debug_mpc` 的 Dashboard 可选接入。
- 控制消息入队，并由主循环安全点消费。

`sp_vision_25` 不负责：

- 浏览器页面。
- 样式或前端脚本。
- 前端依赖 vendor。
- Mosquitto 或 WebSocket MQTT 服务进程。
- HTTP 静态资源服务。

## 依赖

`auto_aim_debug_mpc` 构建或运行 Dashboard MQTT 功能需要：

```bash
sudo apt update
sudo apt install -y libpaho-mqtt-dev libpaho-mqttpp-dev nlohmann-json3-dev
```

如果基础镜像已经提供 `nlohmann/json.hpp`，则 `nlohmann-json3-dev` 不是额外依赖。

Dashboard 服务依赖由独立前端仓库管理。

## 配置方式

`configs/standard3.yaml` 中的配置段：

```yaml
dashboard:
  enabled: false
  robot_id: "myrobot"
  mqtt_host: "tcp://127.0.0.1:1883"
```

默认 `enabled: false`，目的是合入 main 后不改变原有主程序启动行为。

启用方式二选一：

1. 修改部署用 YAML：

```yaml
dashboard:
  enabled: true
  robot_id: "myrobot"
  mqtt_host: "tcp://127.0.0.1:1883"
```

2. 启动时使用 CLI 覆盖：

```bash
./build/auto_aim_debug_mpc --dashboard --robot-id myrobot --mqtt-host tcp://127.0.0.1:1883 configs/standard3.yaml
```

CLI 优先级高于 YAML：

- `--dashboard`：强制启用 Dashboard。
- `--robot-id hero` 或 `--robot-id=hero`：覆盖 `dashboard.robot_id`。
- `--mqtt-host tcp://...` 或 `--mqtt-host=tcp://...`：覆盖 `dashboard.mqtt_host`。

如果 Dashboard 启用但 MQTT 初始化或连接失败，主程序会记录 warning，并继续原视觉业务路径，不直接退出。

## 启动方式

先在独立 Dashboard panel 仓库启动服务：

```bash
docker compose up -d
```

同机部署时，浏览器访问：

```text
http://机器人IP:8080
```

浏览器 MQTT WebSocket：

```text
ws://机器人IP:9001
```

再启动真实 Dashboard 视觉入口：

```bash
./build/auto_aim_debug_mpc --dashboard --mqtt-host tcp://127.0.0.1:1883 configs/standard3.yaml
```

如果 Dashboard 服务在调试电脑上，机器人视觉程序使用对端 LAN IP：

```bash
./build/auto_aim_debug_mpc --dashboard --mqtt-host tcp://调试电脑IP:1883 configs/standard3.yaml
```

## 网络说明

拓扑 A：Dashboard 服务和视觉程序在同一台机器人/主机上。

```text
auto_aim_debug_mpc -> tcp://127.0.0.1:1883
浏览器设备 -> http://机器人IP:8080
浏览器 MQTT WS -> ws://机器人IP:9001
```

这里 C++ 用 `127.0.0.1` 是合理的，因为 broker 与视觉程序在同一网络命名空间；电脑浏览器不能访问自己的 `127.0.0.1:8080`，应访问机器人 IP。

拓扑 B：Dashboard 服务在调试电脑上，视觉程序在机器人上。

```text
auto_aim_debug_mpc -> tcp://调试电脑IP:1883
浏览器设备 -> http://调试电脑IP:8080
浏览器 MQTT WS -> ws://调试电脑IP:9001
```

`127.0.0.1` 不是唯一生产写法；如果 broker 不在视觉程序同一网络命名空间，`--mqtt-host` 必须使用对端 LAN IP。

## 手动验收

有真实硬件和完整依赖时，构建并启动：

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target auto_aim_debug_mpc dashboard_params_test -j$(nproc)
cmake --build build --target standard_mpc -j$(nproc)
./build/auto_aim_debug_mpc --dashboard --mqtt-host tcp://127.0.0.1:1883 configs/standard3.yaml
```

浏览器侧验收在独立 Dashboard panel 仓库执行。

## 合并注意事项

- 合入 main 前应在具备 CMake、OpenVINO、Paho MQTT C++、nlohmann_json 的环境中完成 C++ 构建验证。
- Dashboard PR review 只针对 `auto_aim_debug_mpc` 接入；`standard_mpc` 不作为 Dashboard 使用示例。
- 不应重新引入无硬件开发路径，例如 `--mock-runtime`、`--video-source`、`--video-loop`、mock publisher 或 hardwareless smoke 脚本。
- 真实相机、串口和 CAN 验证应在硬件环境中单独完成。
