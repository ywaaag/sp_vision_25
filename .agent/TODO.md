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
- 2026-04-22 `sentry_omni_perception_debug_mpc` 已将主工业相机链路与双 USB 回退链路解耦：主线程只消费工业相机，双 USB 在独立线程内完成取图、姿态对齐、传统 `Detector`、解算与各自 `Tracker`，主相机不再被 60fps USB 取图节奏限制；USB 回退仍按最近距离选候选目标，在上行 `yaw_diff` 基础上额外叠加左 `+120°` / 右 `-120°` 的固定 yaw 偏置，并保持 `fire=false`。
- 2026-04-21 已新增离线 `buff_detect_test`，默认使用 `configs/standard3.yaml` 检测 `assets/demo.avi` 中的 buff，并在窗口中叠加模型关键点与 `PowerRune` 几何点用于快速目检。
- 2026-04-21 已新增 `tasks/auto_buff_fyt`，将 `rm_rune` 的 YOLOX 能量机关检测、颜色/类型筛选和 R 标修正迁入当前仓库，并保持 `auto_buff` 原有 `Target/Aimer` 预测击打链路复用。
- 2026-04-21 已将 `buff_fyt_detector.confidence_threshold` 收紧到 `0.7`，并把 `buff_detect_fyt_test` 的统计/绘制口径固定为颜色过滤后的候选；实测 `buff_detect_fyt_test --display=false assets/demo.avi` 时整段 `196` 帧中模型命中 `66` 帧，进入原有解算/跟踪链路 `61` 帧。
- 2026-04-21 已新增 `auto_buff_fyt_debug_mpc`，按 `auto_buff_debug_mpc` 结构接入 `auto_buff_fyt` 检测链，并在窗口中叠加 FYT rune 候选框、R 标二值 ROI 与原有 buff 重投影调试信息。
- 2026-04-21 已新增 `standard_fyt_mpc`，按 `standard_mpc` 结构接入 `auto_buff_fyt` 打符检测链，保留原有自瞄、ROS 和 MPC 线程模型；当前已通过编译。
- 2026-04-21 已修正 `auto_buff_fyt` 的 `R` 标传统矫正逻辑：`buff_fyt_detector.min_lightness` 现在真实参与二值化，默认值为 `130`；此前配置项存在但代码实际始终走 `OTSU`。
- 2026-04-21 已新增 `auto_aim_delay_tuner`，用于固定静止装甲板场景下自动左右扫 `±30°` 并比较不同 `imu delay` 对 `target_yaw` 波动的影响，最终输出最优延迟值。
- 2026-04-22 已修正 `tools::Recorder` 的 CFR 录像时间轴：录像现在按真实输入时间戳补帧/限帧，并让 `.avi` 与同名 `.txt` 保持逐帧对齐，避免实际采样帧率低于配置帧率时出现回放加速。
- 2026-04-22 已下调 `buff_fyt_detector.confidence_threshold` 到 `0.5`，并增强 `R` 标修正的先验与轮廓匹配鲁棒性：`assets/demo.avi` 离线统计从 `66/196` 提升到 `78/196`，`assets/big.avi` 从 `95/250` 提升到 `217/250`。
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
- **2026-04-24**: 新增 `omni_perception_delay_tuner`，仿照 `auto_aim_delay_tuner` 的自动扫角评分流程，为左右 USB 感知相机提供独立的姿态延迟标定入口；当前通过 `--camera-side=left|right` 选择相机，并复用 `sentry_omni_perception_debug_mpc` 的 `usb_world_q + Detector + 最近装甲板 + Tracker` 链路。
- **2026-04-24**: 传统 `auto_aim::Detector` 的大小装甲板分型阈值已从 `tasks/auto_aim/detector.cpp` 的硬编码比值接入 YAML。`configs/standard3.yaml` 新增 `small_armor_max_ratio` / `big_armor_min_ratio`，其中当前 `big_armor_min_ratio` 调到 `3.2`，用于减轻 USB 感知相机把小装甲板直接判成 `big` 的情况。
- **2026-04-23**: `auto_aim_delay_tuner` 已扩展为支持 `--camera-source=main|usb_left|usb_right`。主相机继续走 YOLO 检测链；左右 USB 版本走当前仓库的传统 `Detector + Tracker`，并复用 `sentry_omni_perception_debug_mpc` 的 USB 姿态几何假设：yaw/roll 跟随云台、`pitch=0`。
- **2026-04-23**: `sentry_omni_perception_debug_mpc` 的左右感知相机已改为通过 `/dev/usb_cam_left`、`/dev/usb_cam_right` 符号名打开；USB 感知分支的姿态解算改为保留 yaw/roll 并固定 `pitch=0`，避免直接跟随主云台俯仰；新增 `--display` 开关后可同时显示主相机、左 USB、右 USB 三路画面。
- **2026-04-22**: `sentry_omni_perception_debug_mpc` 的 USB 回退下发新增固定左右 yaw 偏置。当前实现会在原有 `gs.yaw_diff` 基础上，针对 `usb_left` 额外叠加 `+120°`、针对 `usb_right` 额外叠加 `-120°`，并继续对 `plan.target_yaw` 与 `plan.yaw` 同步生效。
- **2026-04-22**: 调整 `sentry_omni_perception_debug_mpc` 的线程模型。当前版本将双 USB 的取图、`gimbal.q(...)` 姿态对齐、传统检测、解算和跟踪迁到独立线程，主线程只跑工业相机 YOLO 主链并消费最新 USB 回退结果，避免 165fps 主相机被 60fps USB 阻塞；相关的 `io::Gimbal::q(t)` 也改为基于时间历史缓存的非消费式插值查询，便于多线程同时按时间取姿态。
- **2026-04-22**: 针对 `auto_buff_fyt` 的低识别率问题，下调 `buff_fyt_detector.confidence_threshold` 至 `0.5`，并将 `R` 标修正改为使用同色候选的加权中心作为先验，同时在传统二值化轮廓中优先选包含先验的轮廓、否则回退到最近亮轮廓；离线复测 `assets/demo.avi` 为 `78/196`、`assets/big.avi` 为 `217/250`。
- **2026-04-22**: 修正 `tools::Recorder` 的 AVI 录像时间轴。当前实现改为基于首帧时间戳建立固定输出帧率时间轴，慢于目标帧率时会补写保持帧，避免 `small.avi` / `big.avi` 这类录像因文件头 fps 固定而发生回放加速；同时 `.txt` 旁路姿态文件也改为按输出帧逐行对齐。
- **2026-04-16**: 重整 `.agent/` 文档职责边界，把容器、X11、海康相机、`standard_mpc` / `auto_aim_debug_mpc` 的联调细节下沉到 `.agent/TROUBLESHOOTING.md`，并把 `DEVELOPMENT.md` 收回到开发规范。
- **2026-04-15**: 在 `AGENTS.md` 增加“构建/运行/调试默认先启动并进入 Docker 容器”的元规则，并把当前容器名 `Combat_Sentry2026`、镜像 `combat_sentry_v1:latest` 与进入命令写入 `.agent/DEVELOPMENT.md`。
- **2026-04-15**: 将全向感知 V1 的详细方案下沉到 `.agent/OMNIPERCEPTION_V1.md`，避免 `TODO.md` 混入过多设计细节。
- **2026-04-21**: 新增 `buff_detect_test` 离线测试入口，默认走 `configs/standard3.yaml` 和 `assets/demo.avi`，用于单独验证 `auto_buff` 模型检测并可视化识别结果。
- **2026-04-21**: 新增 `auto_buff_fyt` 模块与 `buff_detect_fyt_test`，移植 `rm_rune` 的 YOLOX 打符识别逻辑并在 `assets/demo.avi` 上完成离线验证。
- **2026-04-21**: 新增 `auto_buff_fyt_debug_mpc` 调试入口，用于在线联调 `auto_buff_fyt` 检测链和原有 MPC 打符链路。
- **2026-04-21**: 新增 `standard_fyt_mpc` 主程序入口，用于在 `standard_mpc` 主线上切换到 `auto_buff_fyt` 打符检测链。

---
