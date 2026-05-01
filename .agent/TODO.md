---

# 开发任务与进度跟踪 (TODO.md)

## 1. 使用方式
- 开始任务前，先看 `当前状态` 与 `当前待办`，再决定是否继续读取 `KNOWLEDGE.md` 或 `TROUBLESHOOTING.md`。
- 若文档与实现冲突，以源码、`configs/*.yaml`、`CMakeLists.txt` 为准，再回写 `.agent/` 文档。
- 本文件只保留任务状态、维护入口和高层归档；详细技术结论应沉淀到 `.agent/KNOWLEDGE.md`。

## 2. 当前状态
- 2026-04-04 已完成一次全局上下文审计，`standard_mpc` 与 `auto_aim_debug_mpc` 的核心知识已同步到 `.agent/`。
- 已沉淀的主题包括：标准 MPC 数据链路、调试模式 JSON 字段、坐标系系统、控制指令语义、下位机通信协议。
- 2026-04-26 MQTT Dashboard 第二阶段 E/F 基线进入合并收尾：Docker smoke 与前端控制面板已对齐，协议裁剪为不承载图像或视频流。
- 2026-04-26 MQTT Dashboard 第二阶段 H 曾实现热调参模型与 Planner/Buff Aimer apply 接口；2026-04-30 范围收缩后当前只保留 Planner 热更新。
- 2026-04-27 MQTT Dashboard 第二阶段 G/H 合并收口完成：`control/cmd` 统一使用 `command` 字段，`DashboardParams` 完整 payload 通过 `MqttBridge::*_payload` 接口发布，I 阶段可接入业务入口。
- 2026-04-27 MQTT Dashboard 前端已融合 `preview.html` 布局，参数面板改为基于 `configs/standard3.yaml` 的 schema 目录，非热参数只读展示。
- 2026-04-27 MQTT Dashboard 生产收口：Dashboard 网络服务与视觉主程序分离，主程序由人工在真实硬件环境启动。
- 2026-04-27 MQTT Dashboard 启动参数已收敛到 `configs/standard3.yaml` 的 `dashboard` 配置段，CLI 仍可覆盖。
- 2026-04-29 MQTT Dashboard PR review 收口：`ThreadSafeQueue` 关闭后会唤醒等待线程，Dashboard CLI/YAML 解析抽到共享 helper，Dashboard 网络服务默认允许局域网访问。
- 2026-04-29 MQTT Dashboard 静态前端曾拆为多文件结构；2026-04-30 后前端成果已迁出视觉仓库。
- 2026-04-29 MQTT Dashboard 前端曾升级为轻量 Panel 工作台；2026-04-30 后浏览器 UI 与服务容器已迁往独立前端仓库。
- 2026-04-29 MQTT Dashboard 合并前边界收口：Dashboard CLI helper 改为 `tools/dashboard_cli.*` 并归入 `dashboard_config`，前端 control topic 映射回收到 `core/protocol.js`，vendor license 信息已补齐。
- 2026-04-30 MQTT Dashboard PR 范围已收缩为 `auto_aim_debug_mpc` only：`standard_mpc` 回到 upstream/main 行为，打符热参链路移除，视觉仓库只保留 MQTT 后端能力。
- 2026-04-30 MQTT Dashboard 前后端分仓收口：`sp_vision_25` 只保留 MQTT 后端能力，浏览器 UI 与 Dashboard 服务容器迁往独立前端仓库。
- 2026-04-30 MQTT Dashboard PR #4 review 修复：`ThreadSafeQueue` 恢复旧阻塞式 `front()`/`pop()` overload，`auto_aim_debug_mpc` 的 Dashboard 接入封装到 `src/auto_aim_debug_dashboard.*`。
- 2026-05-01 MQTT Dashboard PR review 修复：`MqttBridge::start()` 失败路径补齐断连清理，Dashboard 参数 schema/current 改用实际启动配置路径。
- 2026-05-01 MQTT Dashboard 文档收口：`docs/dashboard.md` 成为最终版维护入口，原运行边界、分仓、broker、build、changelog 和协议细节已合并精简。
- 2026-05-01 新增无硬件 Dashboard telemetry smoke 分支：通过独立 `auto_aim_dashboard_hardwareless_test` 读取视频验证算法输出和 MQTT telemetry，不接入生产入口。
- 当前没有明确进行中的功能任务；本文件现阶段主要承担“维护面板”和“回填入口”的作用。
- 历史任务细节已归档到下文，不再在顶部重复展开。

## 3. 当前待办
1. [ ] 在真实运行后回填性能数据（FPS、端到端延迟、CPU、内存），禁止估算。
2. [ ] 若新增算法参数、接口约定或架构结论，更新 `.agent/KNOWLEDGE.md`。
3. [ ] 若新增环境报错、依赖问题或联调坑点，更新 `.agent/TROUBLESHOOTING.md`。
4. [ ] 若再次发现文档与实现不一致，优先核对 `src/standard_mpc.cpp`、`src/auto_aim_debug_mpc.cpp` 和 `configs/*.yaml`。

## 4. 已完成归档

### 2026-04-04 文档校准批次

| ID | 主题 | 状态 | 主要产出 |
| :--- | :--- | :--- | :--- |
| `TASK-000` | 全局架构审计与文档校准 | 已完成 | 校正 `standard_mpc` / `auto_aim_debug_mpc` 与文档的偏差，重写 `.agent/KNOWLEDGE.md` 相关章节，并更新 `.agent/DEVELOPMENT.md` 的构建命令。 |
| `TASK-001` | 标准 MPC 自瞄完整链路分析 | 已完成 | 梳理相机输入、坐标变换、YOLO 检测、跟踪、EKF、规划、控制输出和打符模式的完整数据链路。 |
| `TASK-002` | 调试模式 JSON 字段说明 | 已完成 | 整理 `Plotter` 输出字段的来源、坐标系、物理意义和单位。 |
| `TASK-003` | 坐标系系统梳理 | 已完成 | 归纳 7 个坐标系、关键旋转矩阵、完整变换链和常见易错点。 |
| `TASK-004` | 控制指令与相机-枪口标定澄清 | 已完成 | 明确控制指令的坐标系语义，并补齐手眼标定与弹道补偿链路说明。 |
| `TASK-005` | 下位机通信协议与坐标系澄清 | 已完成 | 补齐串口/CAN 数据结构、四元数语义、控制指令含义与理想坐标系定义。 |

### 本批次关键事实
- EKF 状态向量为 11 维：`[x, vx, y, vy, z, vz, a, ω, r, l, h]`。
- 规划线程调度为：标准模式有目标 `10ms`、空闲 `200ms`；调试模式固定 `10ms`。
- 调试链路包含 JSON 数据记录与重投影可视化。
- 视觉端发送给下位机的 `yaw` / `pitch` 语义已澄清为世界坐标系下的绝对角度。

## 5. 性能事实核查 (Reality Check)
> 仅在真实构建和运行后填写，禁止凭印象补值。

| 指标 | 目标值 | 当前实测值 | 状态 |
| :--- | :--- | :--- | :--- |
| 整体帧率 | 100+ FPS | 待测 | 未回填 |
| 端到端延迟 | < 33ms | 待测 | 未回填 |
| CPU 使用率 | < 70% | 待测 | 未回填 |
| 内存占用 | < 1.5GB | 待测 | 未回填 |

## 6. 更新日志 (Changelog)
- **2026-04-15**: 重构 `TODO.md` 结构，合并重复执行记录，改为“当前状态 + 当前待办 + 已完成归档 + 性能回填”布局。
- **2026-04-27**: 完成 Dashboard G/H 合并审查与接口收口，保持不接入 `src/standard_mpc.cpp` 或 `src/auto_aim_debug_mpc.cpp`。
- **2026-04-27**: 融合 Dashboard 优化前端，并将参数 schema/current 扩展到 `configs/standard3.yaml` 参数目录；复合配置标记为只读、重启生效。
- **2026-04-29**: 完成 Dashboard PR review 收口：队列关闭语义、CLI helper、LAN 默认暴露、Docker 检查脚本和部署文档已同步。
- **2026-04-29**: 曾完成 Dashboard 静态前端解耦；2026-04-30 后相关浏览器资源已迁出视觉仓库。
- **2026-04-29**: 曾完成 Dashboard Panel 工作台升级；2026-04-30 后相关前端实现已迁出视觉仓库。
- **2026-04-29**: 完成 Dashboard 合并前解耦卫生收尾：CLI helper 命名和 CMake 归属收紧、前端 control topic 映射收回协议层、vendor license 补齐。
- **2026-04-30**: 收缩 Dashboard PR 范围：只保留 `auto_aim_debug_mpc` 接入，`standard_mpc` 与 `buff_aimer.*` 回退到 upstream/main，文档补充同机/分离 LAN 拓扑。
- **2026-04-30**: 完成 Dashboard 前后端分仓收口：移除本仓库前端静态资源和 Dashboard 服务容器绑定，新增前端迁移说明。
- **2026-04-30**: 修复 Dashboard PR #4 review：队列兼容旧阻塞 API，修正 MQTT bridge namespace 风格，并封装自瞄调试入口 Dashboard 接入。
- **2026-05-01**: 修复 Dashboard PR review：MQTT start 失败会 best-effort 关闭连接，DashboardParams 不再依赖硬编码 `configs/standard3.yaml`，前端分仓文档去除本机绝对路径。
- **2026-05-01**: 精简 Dashboard 文档：新增 `docs/dashboard.md` 最终版，删除重复的运行、broker、build、分仓和 changelog 文档，保留 `docs/dashboard_mqtt_protocol.md` 作为兼容跳转。
- **2026-05-01**: 新增独立无硬件 Dashboard telemetry smoke 目标：读取 `assets/demo/demo.avi` 验证自瞄算法输出并可选发布 MQTT telemetry，不恢复生产入口 mock runtime。
- **2026-04-26**: 完成 Dashboard 热参数模型 H：新增 `DashboardParams`、Planner/Buff Aimer 热参数快照与单参数 apply，验证范围不包含 TinyMPC Q/R/max_acc 热修改；2026-04-30 后 Buff Aimer 热参链路已按新范围移除。
- **2026-04-15**: 将隐藏知识目录更名为 `.agent/`，并同步更新仓库内所有元规则与文档引用路径。
- **2026-04-15**: 将仓库根 `AGENTS.md` 重写为精简的元规则入口文件，仅保留上下文加载顺序、知识路由与收尾同步要求。
- **2026-04-04**: 完成一次全局架构审计，并沉淀标准链路、调试字段、坐标系、控制指令和通信协议等核心知识。

---
