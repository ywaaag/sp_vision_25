# Dashboard MQTT Protocol

本文件定义视觉程序、Dashboard 前端与后续 `MqttBridge` 共享的 MQTT topic 与 JSON payload 契约。当前 Dashboard 只承载 telemetry、日志、参数和控制消息，不承载图像或视频流。

## Topic 命名

所有 topic 必须带 `{robot_id}` 作为首段前缀，`{robot_id}` 是单个 MQTT topic segment，不应包含 `/`、`+` 或 `#`。

| Topic | Direction | QoS | Payload |
| :--- | :--- | :---: | :--- |
| `{robot_id}/data` | vision -> dashboard | 0 | JSON telemetry |
| `{robot_id}/log` | vision -> dashboard | 0 | JSON log event |
| `{robot_id}/params/schema` | vision -> dashboard | 0 | JSON parameter schema |
| `{robot_id}/params/current` | vision -> dashboard | 0 | JSON current parameter values |
| `{robot_id}/control/param` | dashboard -> vision | 1 | JSON parameter update request |
| `{robot_id}/control/cmd` | dashboard -> vision | 1 | JSON command request |
| `{robot_id}/control/ack` | vision -> dashboard | 1 | JSON request acknowledgement |

QoS 0 用于状态、日志和参数快照，允许丢包并由下一帧或下一次发布覆盖。QoS 1 用于控制请求和响应，确保 Dashboard 与视觉侧都能基于 `request_id` 做幂等处理。

## Payload 通用规则

- JSON 字段名使用 `snake_case`。
- `timestamp` 使用 Unix epoch milliseconds，类型为 integer。
- `request_id` 由请求方生成，在 `control/param`、`control/cmd` 与 `control/ack` 中保持一致。
- 数值单位写入 schema 的 `unit`，普通 payload 不重复携带单位。

## `{robot_id}/data`

用于低频 Dashboard telemetry。`values` 是自由扩展对象，字段含义由后续 Dashboard 或桥接层约定。

`values` 是后续 C++ 发布侧的唯一正式字段。第一阶段前端可临时兼容旧草案中的 `fields` 对象，兼容逻辑只用于读取历史测试消息，不作为后续发布格式。

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

## `{robot_id}/log`

```json
{
  "timestamp": 1770000000000,
  "level": "info",
  "source": "auto_aim_debug_mpc",
  "message": "tracker locked"
}
```

`level` 建议使用 `trace`、`debug`、`info`、`warn`、`error`、`critical`。

## `{robot_id}/params/schema`

参数 schema payload 使用固定 envelope：

```json
{
  "version": 1,
  "params": [
    {
      "key": "camera.exposure",
      "type": "number",
      "min": 0,
      "max": 20000,
      "step": 100,
      "unit": "us",
      "group": "camera",
      "value": 5000,
      "editable": true,
      "restart_required": false
    },
    {
      "key": "tracker.enabled",
      "type": "bool",
      "group": "tracker",
      "value": true,
      "editable": false,
      "restart_required": true
    },
    {
      "key": "runtime.mode",
      "type": "enum",
      "group": "runtime",
      "value": "standard",
      "options": ["standard", "debug"]
    },
    {
      "key": "mqtt.robot_id",
      "type": "string",
      "group": "mqtt",
      "value": "hero"
    }
  ]
}
```

每个参数项支持以下字段：

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

`options` 仅允许在 `type = "enum"` 时使用。`number` 的 `min`、`max`、`step` 可按参数需要提供；非 `number` 参数不使用这些字段。

当前 MPC Dashboard 参数目录以 `configs/standard3.yaml` 为 source of truth 生成。已接入热更新链路的 Planner 参数会设置 `editable: true`，`params/current` 中的值来自运行态快照；其他标量、数组、矩阵、标定参数和打符相关配置只展示 YAML 初值，设置 `editable: false` 与 `restart_required: true`。当前 Dashboard 只接入 `auto_aim_debug_mpc`，不接入 `standard_mpc`。

## `{robot_id}/params/current`

当前参数值快照，`values` 以 schema 中的 `key` 为键。

```json
{
  "timestamp": 1770000000000,
  "values": {
    "camera.exposure": 5000,
    "tracker.enabled": true,
    "runtime.mode": "standard"
  }
}
```

## `{robot_id}/control/param`

Dashboard 请求视觉侧更新单个参数。视觉侧必须用相同 `request_id` 发布 `control/ack`。

```json
{
  "request_id": "8d8b5d6f-6a0b-4ef1-9d75-43e0e6f51a10",
  "key": "camera.exposure",
  "value": 6000,
  "timestamp": 1770000000000
}
```

## `{robot_id}/control/cmd`

Dashboard 请求视觉侧执行命令。`args` 为命令参数对象，无参数时使用空对象。

```json
{
  "request_id": "3d37909b-3c14-45e8-bb62-4b4b301f9a13",
  "command": "snapshot",
  "args": {},
  "timestamp": 1770000000000
}
```

`args` 可省略；视觉侧收到缺省 `args` 时必须按空对象 `{}` 处理。

命令集合由后续 `MqttBridge` 或业务接入层扩展，本阶段只固定 envelope。

## `{robot_id}/control/ack`

`control/ack` 是所有控制请求的统一响应格式。

```json
{
  "request_id": "8d8b5d6f-6a0b-4ef1-9d75-43e0e6f51a10",
  "ok": true,
  "message": "camera.exposure applied",
  "applied": {
    "camera.exposure": 6000
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
