# MQTT Dashboard Final Guide

本文件是 `sp_vision_25` 中 MQTT Dashboard 相关内容的最终维护入口，合并了原运行边界、构建说明、broker 拓扑、前端分仓和协议说明。当前仓库只保留视觉程序侧 MQTT 后端能力，不再包含浏览器 UI、CSS、JS、vendor、Mosquitto 容器或 HTTP 静态服务。

## Current Scope

`sp_vision_25` 负责：

- MQTT topic 与 JSON payload 契约。
- C++ `MqttBridge`。
- Dashboard 参数 schema/current 生成。
- Dashboard YAML/CLI 配置解析。
- `auto_aim_debug_mpc` 的可选 Dashboard 接入。

`sp_vision_25` 不负责：

- Dashboard browser UI。
- CSS、JS、前端 vendor assets。
- Mosquitto、MQTT over WebSocket 或 HTTP static server。
- 生产入口 mock runtime、mock publisher。
- Dashboard 网络传输中的 image/video/MJPEG/Web terminal。

Dashboard 当前只接入 `src/auto_aim_debug_mpc.cpp`。`src/standard_mpc.cpp` 保持 upstream/main 行为，不链接 `mqtt_bridge`、`dashboard_params`、`dashboard_config`，也不定义 `SP_VISION_ENABLE_DASHBOARD_MQTT`。

## Repository Split

前端与服务栈应由独立仓库维护，例如：

```text
sp_vision_dashboard_panel
```

该独立仓库负责：

- 提供 HTML/CSS/JS 和 browser-side panel。
- 提供 Mosquitto native MQTT `1883`。
- 提供 MQTT over WebSocket `9001`。
- 提供 HTTP static Dashboard `8080`。
- 不包含视觉算法代码。
- 只通过本文件定义的 MQTT topic/payload 与 `sp_vision_25` 通信。

推荐结构：

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

`/tmp/sp_vision_dashboard_panel_seed/` 只是迁移时的临时种子目录，不是长期方案，也不应写入本仓库。

## Build Integration

根 `CMakeLists.txt` 引入：

```cmake
include("${PROJECT_SOURCE_DIR}/cmake/DashboardDeps.cmake")
```

`cmake/DashboardDeps.cmake` 只做可选探测，不应让普通视觉构建强依赖 Dashboard：

- `find_package(PahoMqttCpp QUIET)` 探测 MQTT C++ 支持。
- 如果 Paho MQTT C++ 不存在，非 Dashboard 视觉目标仍应能 configure。
- 如果 `mqtt_bridge` target 存在，只有 `auto_aim_debug_mpc` 链接 `mqtt_bridge` 与 `dashboard_params`。
- 只有 `auto_aim_debug_mpc` 定义 `SP_VISION_ENABLE_DASHBOARD_MQTT`。

Dashboard MQTT 后端需要：

```bash
sudo apt update
sudo apt install -y libpaho-mqtt-dev libpaho-mqttpp-dev nlohmann-json3-dev
```

如果基础环境已提供 `nlohmann/json.hpp`，`nlohmann-json3-dev` 不是额外要求。

## Vision Configuration

`configs/standard3.yaml` 中保留 Dashboard 配置段：

```yaml
dashboard:
  enabled: false
  robot_id: "myrobot"
  mqtt_host: "tcp://127.0.0.1:1883"
```

默认 `enabled: false`，合入 main 后不改变原有启动行为。

启用方式可以选 YAML：

```yaml
dashboard:
  enabled: true
  robot_id: "myrobot"
  mqtt_host: "tcp://127.0.0.1:1883"
```

也可以用 CLI 覆盖：

```bash
./build/auto_aim_debug_mpc --dashboard --robot-id myrobot --mqtt-host tcp://127.0.0.1:1883 configs/standard3.yaml
```

CLI 优先级高于 YAML：

- `--dashboard`：强制启用 Dashboard。
- `--robot-id hero` 或 `--robot-id=hero`：覆盖 `dashboard.robot_id`。
- `--mqtt-host tcp://...` 或 `--mqtt-host=tcp://...`：覆盖 `dashboard.mqtt_host`。

如果 Dashboard 启用但 MQTT 初始化或连接失败，视觉程序应记录 warning 并降级关闭 Dashboard telemetry，不直接退出主视觉业务。

## Deployment

先在独立 Dashboard panel 仓库启动服务：

```bash
docker compose up -d
```

独立 Dashboard 服务应提供：

| Port | Purpose |
| :--- | :--- |
| `1883/tcp` | native MQTT for C++ vision client |
| `9001/tcp` | MQTT over WebSocket for browser client |
| `8080/tcp` | browser Dashboard HTTP |

启动视觉程序：

```bash
./build/auto_aim_debug_mpc --dashboard --mqtt-host tcp://127.0.0.1:1883 configs/standard3.yaml
```

没有 Dashboard 时的正常业务入口保持不变，例如：

```bash
./build/standard_mpc configs/standard3.yaml
./build/auto_aim_debug_mpc configs/standard3.yaml
```

不要在没有 camera、gimbal、CAN 和模型资源的机器上宣称完成真实运行验收。

## Network Topologies

拓扑 A：Dashboard 服务和视觉程序在同一台机器人或主机上。

```text
auto_aim_debug_mpc -> tcp://127.0.0.1:1883
browser device -> http://robot-lan-ip:8080
browser MQTT WS -> ws://robot-lan-ip:9001
```

这里 C++ 使用 `127.0.0.1` 是因为 broker 与视觉程序在同一网络命名空间。电脑或平板浏览器不能访问自己的 `127.0.0.1:8080`，必须访问机器人 LAN IP。

拓扑 B：Dashboard 服务在调试电脑上，视觉程序在机器人上。

```text
auto_aim_debug_mpc -> tcp://debug-pc-lan-ip:1883
browser device -> http://debug-pc-lan-ip:8080
browser MQTT WS -> ws://debug-pc-lan-ip:9001
```

如果 broker 不在视觉程序同一网络命名空间，`dashboard.mqtt_host` 或 `--mqtt-host` 必须使用 Dashboard 主机 LAN IP。

## Runtime Safety

- C++ callback 不直接修改业务状态，控制消息进入队列，由主循环安全点消费。
- `MqttBridge::start()` 失败后应 best-effort 关闭队列、join publish thread 并断开 MQTT client。
- `stop_dashboard` 只关闭 telemetry 发布，不停止视觉主程序。
- `start_dashboard` 恢复 telemetry 发布。
- `republish_params` 重新发布 `params/schema` 和 `params/current`。

## Manual Validation

有完整依赖时：

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target auto_aim_debug_mpc dashboard_params_test -j$(nproc)
cmake --build build --target standard_mpc -j$(nproc)
```

宿主机缺少 OpenVINO、Paho 或其他 C++ 依赖时，可以使用本仓库的开发容器构建视觉侧目标：

```bash
docker compose -f docker-compose.dev.yml build
docker compose -f docker-compose.dev.yml run --rm sp-vision-dev \
  bash -lc 'cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build --target auto_aim_debug_mpc dashboard_params_test -j"$(nproc)"'
```

如果 Paho 可用，还可以构建：

```bash
cmake --build build --target mqtt_bridge_smoke -j$(nproc)
```

前端、broker、WebSocket 和 HTTP 验收在独立 Dashboard panel 仓库执行。

## Hardwareless Telemetry Smoke

如果目标是在没有下位机、相机和 gimbal 的情况下观察自瞄算法链路是否能跑通，并验证 Dashboard telemetry，可以使用独立测试目标：

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target auto_aim_dashboard_hardwareless_test -j$(nproc)
```

也可以在开发容器中构建：

```bash
./scripts/hardwareless_dashboard_build.sh
```

先在独立 Dashboard panel 仓库启动 broker、WebSocket 与 HTTP 服务：

```bash
cd /home/ywag/sp_vision_dashboard_panel
./scripts/dashboard_up.sh
```

再回到本仓库运行视频输入测试：

```bash
./build/auto_aim_dashboard_hardwareless_test \
  --dashboard \
  --mqtt-host tcp://127.0.0.1:1883 \
  --video-source assets/demo/demo.avi \
  --video-loop \
  --max-frames 300 \
  configs/sentry_pose.yaml
```

容器运行等价命令：

```bash
./scripts/hardwareless_dashboard_run.sh
```

浏览器访问：

```text
http://robot-lan-ip:8080
```

页面填写：

```text
Broker URL: ws://robot-lan-ip:9001
Robot ID: myrobot
```

该目标只读取视频文件和同名姿态文本，例如 `assets/demo/demo.avi` 与 `assets/demo/demo.txt`。如果姿态文本不存在，会使用 identity gimbal pose。它不构造 `io::Camera` 或 `io::Gimbal`，也不修改 `standard_mpc` 或真实 `auto_aim_debug_mpc` 的启动逻辑。

注意：该目标是无硬件算法与 Dashboard telemetry smoke，不等价于真实链路验收。若要实际发布 MQTT telemetry，构建环境仍需要 Paho MQTT C/C++，否则 Dashboard helper 会按降级逻辑记录 warning 并关闭 telemetry 发布。

合并前建议检查：

```bash
git diff --check
git diff --exit-code upstream/main -- src/standard_mpc.cpp
rg -n "gridstack|echarts|mqtt.min.js|dashboard/css|dashboard/js|dashboard/vendor" .
rg -n "mock-runtime|video-source|video-loop|VideoFrameSource|hardwareless|mock_publisher|MJPEG|mjpeg|/image" .
```

前端残留扫描不应命中实际前端源码或 vendor。mock/image/video 扫描中，`video-source` 与 `hardwareless` 只应命中独立 `auto_aim_dashboard_hardwareless_test`、配套脚本和文档；不允许命中生产入口 mock runtime、mock publisher 或图像/视频传输功能。

## MQTT Protocol

所有 topic 必须带 `{robot_id}` 作为首段前缀。`{robot_id}` 是单个 MQTT topic segment，不应包含 `/`、`+` 或 `#`。

| Topic | Direction | QoS | Payload |
| :--- | :--- | :---: | :--- |
| `{robot_id}/data` | vision -> dashboard | 0 | JSON telemetry |
| `{robot_id}/log` | vision -> dashboard | 0 | JSON log event |
| `{robot_id}/params/schema` | vision -> dashboard | 0 | JSON parameter schema |
| `{robot_id}/params/current` | vision -> dashboard | 0 | JSON current parameter values |
| `{robot_id}/control/param` | dashboard -> vision | 1 | JSON parameter update request |
| `{robot_id}/control/cmd` | dashboard -> vision | 1 | JSON command request |
| `{robot_id}/control/ack` | vision -> dashboard | 1 | JSON request acknowledgement |

QoS 0 用于状态、日志和参数快照，允许丢包并由下一帧或下一次发布覆盖。QoS 1 用于控制请求和响应，Dashboard 与视觉侧都基于 `request_id` 做关联和幂等处理。

通用规则：

- JSON 字段名使用 `snake_case`。
- `timestamp` 使用 Unix epoch milliseconds，类型为 integer。
- `request_id` 由请求方生成，在 `control/param`、`control/cmd` 与 `control/ack` 中保持一致。
- 数值单位写入 schema 的 `unit`，普通 payload 不重复携带单位。

### `{robot_id}/data`

`values` 是正式 telemetry 字段。历史测试消息中的 `fields` 只允许前端作为 fallback 读取，不作为 C++ 发布格式。

```json
{
  "timestamp": 1770000000000,
  "values": {
    "fps": 120.5,
    "target_found": true,
    "yaw": 1.25,
    "pitch": -0.42
  }
}
```

### `{robot_id}/log`

```json
{
  "timestamp": 1770000000000,
  "level": "info",
  "source": "auto_aim_debug_mpc",
  "message": "tracker locked"
}
```

`level` 建议使用 `trace`、`debug`、`info`、`warn`、`error`、`critical`。

### `{robot_id}/params/schema`

```json
{
  "version": 1,
  "params": [
    {
      "key": "planner.yaw_offset_deg",
      "type": "number",
      "min": -20,
      "max": 20,
      "step": 0.1,
      "unit": "deg",
      "group": "planner",
      "value": 0,
      "editable": true,
      "restart_required": false
    },
    {
      "key": "camera.exposure",
      "type": "number",
      "group": "camera",
      "value": 5000,
      "editable": false,
      "restart_required": true
    }
  ]
}
```

每个参数项支持：

| Field | Required | Description |
| :--- | :---: | :--- |
| `key` | yes | 参数唯一键，建议使用点分层级 |
| `type` | yes | `number` / `bool` / `enum` / `string` |
| `min` | number only | `number` 参数最小值 |
| `max` | number only | `number` 参数最大值 |
| `step` | number only | `number` 参数调整步长 |
| `unit` | optional | 显示单位，例如 `us`、`ms`、`deg` |
| `group` | yes | Dashboard 分组名 |
| `value` | yes | 当前值，类型必须与 `type` 匹配 |
| `options` | enum only | `enum` 的可选字符串数组 |
| `editable` | optional | `false` 表示 Dashboard 只展示，不发送 `control/param` |
| `restart_required` | optional | `true` 表示当前值来自配置快照，修改需要重启或后续专门热更新链路 |
| `render` | optional | 复合配置可用 `json`，Dashboard 按只读 JSON 文本展示 |
| `source_key` | optional | 运行态别名对应的配置源键 |

当前参数目录由 `auto_aim_debug_mpc` 实际启动的 config path 生成。已接入热更新链路的 Planner 参数设置 `editable: true`，`params/current` 中的值来自运行态快照；其他标量、数组、矩阵、标定参数和打符相关配置只展示 YAML 初值，设置 `editable: false` 与 `restart_required: true`。

### `{robot_id}/params/current`

```json
{
  "timestamp": 1770000000000,
  "values": {
    "planner.yaw_offset_deg": 0,
    "camera.exposure": 5000
  }
}
```

### `{robot_id}/control/param`

```json
{
  "request_id": "8d8b5d6f-6a0b-4ef1-9d75-43e0e6f51a10",
  "key": "planner.yaw_offset_deg",
  "value": 1.5,
  "timestamp": 1770000000000
}
```

视觉侧必须用相同 `request_id` 发布 `control/ack`。

### `{robot_id}/control/cmd`

```json
{
  "request_id": "3d37909b-3c14-45e8-bb62-4b4b301f9a13",
  "command": "republish_params",
  "args": {},
  "timestamp": 1770000000000
}
```

`args` 可省略；视觉侧收到缺省 `args` 时必须按空对象 `{}` 处理。

当前已接入命令：

| Command | Effect |
| :--- | :--- |
| `stop_dashboard` | 停止 Dashboard telemetry 发布 |
| `start_dashboard` | 恢复 Dashboard telemetry 发布 |
| `republish_params` | 重新发布 `params/schema` 与 `params/current` |

### `{robot_id}/control/ack`

```json
{
  "request_id": "8d8b5d6f-6a0b-4ef1-9d75-43e0e6f51a10",
  "ok": true,
  "message": "planner.yaw_offset_deg applied",
  "applied": {
    "planner.yaw_offset_deg": 1.5
  },
  "timestamp": 1770000000000
}
```

| Field | Required | Description |
| :--- | :---: | :--- |
| `request_id` | yes | 对应 `control/param` 或 `control/cmd` 的请求 ID |
| `ok` | yes | 请求是否成功应用 |
| `message` | yes | 面向 Dashboard 的简短结果说明 |
| `applied` | yes | 已实际应用的参数或命令结果对象；失败时可为空对象 |
| `timestamp` | yes | 视觉侧生成 ack 的 Unix epoch milliseconds |

## Contract Source

本文件和 `tools/dashboard_mqtt_contract.hpp` 是 Dashboard 契约源。前端仓库可以复制本节内容，但不能绕过这里直接改变 topic、payload、QoS 或控制语义。
