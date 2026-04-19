---

# 开发任务与进度跟踪 (TODO.md)

## 1. 文档职责
- 本文档只维护当前状态、待办、归档和最近同步结果。
- 开发规范、容器进入方式和标准运行命令看 `.agent/DEVELOPMENT.md`。
- 架构、协议和坐标链事实看 `.agent/KNOWLEDGE.md`。
- 具体报错、联调记录和排障路径看 `.agent/TROUBLESHOOTING.md`。

## 2. 当前状态
- 2026-04-15 已完成 `gimbal` 串口协议和控制语义的一轮源码校准。
- 2026-04-15 已明确：构建、运行、调试默认在 Docker 容器 `Combat_Sentry2026` 内完成。
- 2026-04-16 已确认 `standard_mpc configs/standard3.yaml` 可以在当前容器内正常启动，当前程序是无 GUI 的 headless 主程序。
- 2026-04-16 已确认 `auto_aim_debug_mpc configs/standard3.yaml` 的主要显示问题来自宿主机 X11 授权；在宿主机执行 `xhost +SI:localuser:root` 后，程序已能在当前容器内正常启动并持续运行。
- 2026-04-16 当前联调焦点已经从“容器/X11/GTK 初始化失败”转移到“自瞄链路行为本身”，终端已出现 `[Tracker] Target diverged!` 等算法层日志。
- 2026-04-15 已形成“全向感知 V1”方案，但 `tasks/omniperception` 现状仍是原型，不能直接视为串口主线可用实现。
- `yaw_diff` 当前只透传和调试输出，尚未进入上层闭环；其真实单位、方向和零点仍需实测。

## 3. 当前待办
### 3.1 高优先级联调
1. 在“单进程独占相机”的前提下继续排查 `auto_aim_debug_mpc` 的跟踪发散问题，重点看检测、Tracker 状态机和参数配置。


### 3.2 中长期任务
1. 新建 `omnidirectional_mpc`，保持 `standard_mpc` 不动，把全向感知 V1 挂在串口主线上独立验证。
2. 扩展三相机配置和双层 yaw 几何层，预留 `yaw_diff` 映射，当前先允许固定夹角占位。
3. 跑通左右 USB 的独立 `detector + solver + tracker`，并把切枪逻辑接入主流程，切换阶段不开火。
4. 实测标定 `yaw_diff`、左右 USB 外参和切枪链路；详细步骤与验收项统一维护在 `.agent/OMNIPERCEPTION_V1.md`。

## 4. 已完成归档

| ID | 主题 | 状态 | 主要产出 |
| :--- | :--- | :--- | :--- |
| `TASK-000` | `.agent/` 全局校准 | 已完成 | 重新核对入口程序、构建目标、工作链路与 `.agent/` 文档的一致性。 |
| `TASK-001` | gimbal 串口协议核查 | 已完成 | 核清 `GimbalToVision` / `VisionToGimbal` / `NavToGimbal` 的字段、包头、尾字节、校验和与上层使用方式。 |
| `TASK-002` | gimbal 控制语义校准 | 已完成 | 澄清 `q` 的插值用法、`bullet_count` 的用途、`yaw/pitch` 控制字段的来源，以及 `pitch` 控制符号约定。 |
| `TASK-003` | 容器图形联调链路打通 | 已完成 | 确认当前 `Combat_Sentry2026` 具备图形/USB/网络基础挂载，并验证 `xhost +SI:localuser:root` 可以让 `auto_aim_debug_mpc` 正常启动。 |

## 5. 待回填实测
> 仅在真实构建和运行后填写，禁止凭印象补值。

暂无

## 6. 最近同步
- **2026-04-16**: 重整 `.agent/` 文档职责边界，把容器、X11、海康相机、`standard_mpc` / `auto_aim_debug_mpc` 的联调细节下沉到 `.agent/TROUBLESHOOTING.md`，并把 `DEVELOPMENT.md` 收回到开发规范。
- **2026-04-15**: 在 `AGENTS.md` 增加“构建/运行/调试默认先启动并进入 Docker 容器”的元规则，并把当前容器名 `Combat_Sentry2026`、镜像 `combat_sentry_v1:latest` 与进入命令写入 `.agent/DEVELOPMENT.md`。
- **2026-04-15**: 将全向感知 V1 的详细方案下沉到 `.agent/OMNIPERCEPTION_V1.md`，避免 `TODO.md` 混入过多设计细节。

---
