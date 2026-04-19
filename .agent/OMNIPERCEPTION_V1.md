---

# 全向感知 V1 详细实施方案 (OMNIPERCEPTION_V1.md)

## 1. 目标与当前约束

### 1.1 最终目标
- 在串口主线旁新增一个 `omnidirectional_mpc` 入口，支持三相机联合工作。
- 主工业相机固定在小 yaw 和枪口上，继续负责主自瞄闭环。
- 左右 USB 相机固定在大 yaw 上，覆盖背向或侧向视野。
- 当主工业相机没有目标时，从左右 USB 的候选目标中选择一个，让枪口和主工业相机转向该目标。

### 1.2 首版只做什么
- 先做几何底座、三路独立感知链和切枪逻辑。
- 切换期间只转枪，不开火。
- `yaw_diff` 先允许使用固定角度占位，同时保留未来接入真实值的接口。
- 保持 `standard_mpc` 不动，避免影响当前已有链路。

### 1.3 首版明确不做什么
- 不在 V1 里直接复活旧 `sentry` / `sentry_multithread` 作为正式主线。
- 不做跨相机全局 EKF、复杂多目标融合和复杂决策树。
- 不让 USB 相机直接触发开火。
- 不默认相信下位机返回的 `yaw_diff` 已经可直接用于控制。

## 2. 当前仓库事实

### 2.1 当前主链路
- 串口主程序是 `src/standard_mpc.cpp`。
- 工业相机链路是：
  - `io::Camera::read(img, t)`
  - `io::Gimbal::q(t - 6ms)`
  - `auto_aim::Solver::set_R_gimbal2world(q)`
  - `YOLO::detect(img)`
  - `Tracker::track(...)`
  - `Planner::plan(...)`
  - `gimbal.send(...)`

### 2.2 当前 omniperception 代码的真实状态
- `tasks/omniperception/decider.cpp` 主要按相机名和硬编码视场偏置估算 `delta_yaw`。
- 这套逻辑没有把双层 yaw 几何、左右 USB 到枪口的外参和 `yaw_diff` 严格接进来。
- 因此它只能算原型验证，不适合作为正式几何求解层继续累加功能。

### 2.3 当前 `yaw_diff` 的真实状态
- `yaw_diff` 是 `GimbalToVision` 上行包中的正式字段。
- 读线程会把它写入 `GimbalState`。
- 当前仓库里它只用于调试输出，没有进入 `Planner`、`Tracker`、`Aimer` 或 `standard_mpc` 的控制闭环。
- 现阶段不能假设：
  - 单位已经是角度
  - 正负方向已经明确
  - 零点已经和机械零位对齐

### 2.4 当前单相机假设
- `auto_aim::Solver` 默认读取一套全局：
  - `camera_matrix`
  - `distort_coeffs`
  - `R_camera2gimbal`
  - `t_camera2gimbal`
- 这意味着当前代码天然假设“只有一个用于求解的相机”。
- 若要接三相机，必须让 `Solver` 或其等价封装支持每个相机独立参数实例。

## 3. V1 总体设计决定

### 3.1 入口策略
- 新建 `src/omnidirectional_mpc.cpp`。
- 该入口沿用 `standard_mpc` 的串口云台、主自瞄和 Buff 主体结构，但把全向感知逻辑作为一层新模块接入。
- `standard_mpc` 和 `auto_aim_debug_mpc` 保持不动，继续作为当前可用链路和调试基线。

### 3.2 三相机策略
- 主工业相机：
  - 继续使用 YOLO
  - 继续承担唯一开火闭环
- 左 USB：
  - 使用传统 `Detector`
  - 独立 `Solver`
  - 独立 `Tracker`
- 右 USB：
  - 使用传统 `Detector`
  - 独立 `Solver`
  - 独立 `Tracker`

### 3.3 切换策略
- 主工业相机有目标时：
  - 完全沿用主工业相机自瞄链路
  - USB 相机只做后台候选
- 主工业相机无目标时：
  - 从左右 USB 的候选目标中选一个
  - 选择规则以装甲板优先级为第一依据
  - 若优先级相同，再用所需转角更小作为第二依据
- 切换阶段：
  - 只转向
  - `fire=false`
- 当主工业相机重新看见同目标或更高优先级目标后：
  - 立即切回主工业相机链路

### 3.4 `yaw_diff` 策略
- 当前先保留两套来源：
  - 固定占位角 `big_yaw_fixed_offset`
  - 实时上行 `gs.yaw_diff`
- V1 默认走固定占位角。
- 接口必须支持后续通过配置直接切换到真实 `yaw_diff` 映射，而不用重改主流程。

## 4. 推荐的软件结构

### 4.1 新增模块划分
- `OmniGeometry`
  - 统一处理双层 yaw 几何、相机安装外参和从任一相机观测到枪口目标角的变换
- `OmniCameraPipeline`
  - 封装单个相机的一整套感知链
  - 对主工业相机和 USB 相机可以有不同实现
- `OmniTargetCandidate`
  - 表示某一路相机当前给出的候选目标
- `OmniSwitcher`
  - 实现主相机与 USB 候选之间的切换策略

### 4.2 `OmniGeometry` 的职责
- 输入：
  - 主云台姿态四元数 `q`
  - 当前 `GimbalState`
  - 某个相机的安装外参
  - 某个目标在该相机链路下求得的三维位置或目标 yaw/pitch
- 输出：
  - 当前小 yaw 朝向
  - 当前大 yaw 朝向
  - 该目标相对枪口的目标 yaw/pitch
- 该模块不能混入目标优先级、状态机或开火逻辑。

### 4.3 `OmniCameraPipeline` 的职责
- 每个实例只负责一台相机。
- 至少包含：
  - 图像读取
  - 检测
  - `Solver`
  - `Tracker`
  - 候选目标输出
- 对外只暴露：
  - 当前是否有有效目标
  - 最佳候选目标
  - 调试信息

### 4.4 `OmniTargetCandidate` 建议字段
- `source_camera`
- `timestamp`
- `has_target`
- `armor_name`
- `priority`
- `xyz_in_mount` 或等价三维位置
- `target_yaw`
- `target_pitch`
- `tracking_state`
- `requires_switch`

## 5. 配置设计

### 5.1 为什么必须重构配置
- 当前 YAML 只表达一套相机内外参。
- 三相机方案要求主工业相机、左 USB、右 USB 各自有不同的：
  - 内参
  - 畸变
  - 安装旋转
  - 安装平移
- 若继续共用当前全局键名，会导致三路 `Solver` 互相污染。

### 5.2 建议配置块
- `main_camera`
- `usb_left`
- `usb_right`
- `omni_geometry`
- `omni_switch`

### 5.3 每个相机配置至少应有
- `enabled`
- `camera_type`
- `device_name` 或相机选择标识
- `camera_matrix`
- `distort_coeffs`
- `R_camera_to_mount`
- `t_camera_to_mount`
- 相机所在安装层级：
  - 主工业相机挂在 `small_yaw`
  - 左右 USB 挂在 `big_yaw`

### 5.4 `omni_geometry` 建议参数
- `yaw_diff_enabled`
- `yaw_diff_sign`
- `yaw_diff_scale`
- `yaw_diff_zero`
- `big_yaw_fixed_offset`
- `small_yaw_to_muzzle_offset`
- `big_yaw_to_small_yaw_reference`

### 5.5 `omni_switch` 建议参数
- `enable_omni_switch`
- `same_priority_angle_weight`
- `switch_confirm_count`
- `main_retake_confirm_count`
- `lost_timeout_ms`

### 5.6 V1 默认值建议
- `yaw_diff_enabled: false`
- `big_yaw_fixed_offset`: 先填当前已知机械安装角
- `switch_confirm_count`: 小值，优先保证能切换
- `main_retake_confirm_count`: 小值，优先保证主工业相机尽快接管

## 6. `yaw_diff` 处理规范

### 6.1 统一解释接口
- 无论来源是固定角还是实时上行，几何层都应该最终输出一个统一量：
  - `big_minus_small_yaw_rad`

### 6.2 映射规则
- 当 `yaw_diff_enabled=false`：
  - `big_minus_small_yaw_rad = big_yaw_fixed_offset`
- 当 `yaw_diff_enabled=true`：
  - `big_minus_small_yaw_rad = yaw_diff_sign * yaw_diff_scale * (raw_yaw_diff - yaw_diff_zero)`

### 6.3 当前不能做的假设
- 不能假设 `raw_yaw_diff` 的单位是角度。
- 不能假设正方向就是数学正方向。
- 不能假设零点和机械正前方一致。

### 6.4 联调时必须记录
- `yaw_diff_raw`
- `yaw_diff_mapped`
- `big_yaw_fixed_offset`
- `command_yaw`
- 当前选中的来源相机

## 7. 三路感知链的实现要求

### 7.1 主工业相机链
- 尽量复用现有 `standard_mpc`：
  - `Camera`
  - `YOLO`
  - `Solver`
  - `Tracker`
  - `Planner`
- 主工业相机链路仍然是唯一的正式开火链。

### 7.2 USB 链路
- 左右 USB 相机各自维护：
  - `USBCamera`
  - `Detector`
  - 独立 `Solver`
  - 独立 `Tracker`
- USB 链路的目标输出不应该再停留在“FOV 偏置算一个 `delta_angle`”。
- USB 链路必须给出经过外参和双层 yaw 几何转换后的枪口目标角。

### 7.3 `Solver` 的改造方向
- 推荐把 `Solver` 变成“构造时传入相机参数”的实例化对象。
- 不要再让三路相机共用同一组全局相机参数。
- 如果不想直接重构现有 `Solver`，可以新增一个薄封装，把 YAML 中单独一台相机的参数读出来后再构造内部实例。

### 7.4 Tracker 的使用原则
- 主工业相机、左 USB、右 USB 都各自维护自己的 `Tracker`。
- 不做“一个全局 Tracker 管三路相机”的设计。
- 首版先让每路 Tracker 只服务该路相机的局部时间序列，避免跨相机时序和量测模型立刻复杂化。

## 8. 主程序状态机

### 8.1 建议状态
- `main_tracking`
- `omni_switching`
- `lost`

### 8.2 状态定义
- `main_tracking`
  - 主工业相机当前有可用目标
  - 输出走现有 `Planner`
  - 是否开火由现有主链逻辑决定
- `omni_switching`
  - 主工业相机无目标
  - USB 至少有一个候选目标
  - 输出由几何层给出目标 yaw/pitch
  - 强制 `fire=false`
- `lost`
  - 三路都没有可用目标
  - 不控制或发送安全空命令

### 8.3 状态切换规则
- `main_tracking -> omni_switching`
  - 主工业相机连续若干帧无目标
  - USB 至少一侧出现稳定候选
- `omni_switching -> main_tracking`
  - 主工业相机重新观测到同目标或更高优先级目标
- `omni_switching -> lost`
  - USB 候选全部失效
- `lost -> main_tracking`
  - 主工业相机重新找到目标
- `lost -> omni_switching`
  - 主工业相机仍无目标，但 USB 找到稳定候选

## 9. 目标选择规则

### 9.1 一级排序
- 装甲板优先级。
- 直接沿用现有项目中的装甲板优先级规则，避免同时引入新规则和新几何链。

### 9.2 二级排序
- 若左右 USB 目标优先级相同，比较切枪所需转角。
- 所需转角更小者优先。

### 9.3 三级约束
- 若某路候选虽然存在，但几何变换后超出合理转向范围，则丢弃。
- 若某路候选时间戳过旧，则丢弃。
- 若某路 Tracker 处于明显不稳定状态，则丢弃。

### 9.4 V1 中不做的选择逻辑
- 不比较跨相机历史命中率。
- 不比较复杂时间代价模型。
- 不根据导航或裁判系统额外信息动态改变切换规则。

## 10. 实施阶段与建议顺序

### 阶段 A：先把配置结构改出来
- 先设计 YAML 结构。
- 先让程序能同时读取三相机参数和双层 yaw 参数。
- 这一阶段不改控制逻辑，只做配置和对象构造准备。

### 阶段 B：单独实现双层 yaw 几何层
- 新建 `OmniGeometry`。
- 先使用固定占位角跑通。
- 让它输出稳定的 `big_minus_small_yaw_rad` 和目标枪口角。

### 阶段 C：让左右 USB 链路先独立跑起来
- 先只做：
  - 读图
  - 检测
  - 求解
  - 跟踪
  - 调试输出
- 先不要参与下发控制。
- 先确认左右 USB 的输出在静态目标上是稳定的。

### 阶段 D：新增 `omnidirectional_mpc`
- 复制 `standard_mpc` 的骨架。
- 把三路链路统一接入。
- 先让状态机只在日志中切换，不立刻发真实切枪指令。

### 阶段 E：让 USB 候选驱动切枪
- 在 `omni_switching` 状态下开始下发 yaw/pitch。
- 强制 `fire=false`。
- 确认切枪方向正确、控制值连续、不会来回抖动。

### 阶段 F：联调回填真实 `yaw_diff`
- 记录原始 `yaw_diff` 和实际机械响应。
- 标定方向、比例和零点。
- 切换 `yaw_diff_enabled=true`。
- 再次验证切枪方向和角度是否正确。

## 11. 调试与日志要求

### 11.1 必须记录的量
- `yaw_diff_raw`
- `yaw_diff_mapped`
- `big_yaw_fixed_offset`
- `main_target_yaw`
- `usb_left_target_yaw`
- `usb_right_target_yaw`
- `command_yaw`
- `command_pitch`
- `switch_source`
- `omni_state`

### 11.2 联调时建议看的问题
- 左右 USB 的目标角是否符号一致。
- 从背向目标切到正向目标时，命令 yaw 是否连续。
- `yaw_diff` 切换为真实值后，方向是否反了。
- 主工业相机重新接管时是否有控制跳变。

## 12. 验收标准

### 12.1 代码结构验收
- `standard_mpc` 保持可用。
- 新增 `omnidirectional_mpc` 可独立构建。
- 三路相机参数互不污染。

### 12.2 功能验收
- 主工业相机有目标时，系统仍按主自瞄工作。
- 主工业相机无目标时，系统能从左右 USB 中选出候选目标。
- 切换时能输出合理的目标 yaw/pitch。
- 切换期间不会开火。
- 主工业相机重新找到目标后能顺利接管。

### 12.3 联调验收
- 记录到了 `yaw_diff` 原始值和映射值。
- 记录到了左右 USB 目标角和最终命令角。
- 至少完成一次固定占位角版本的切枪验证。
- 至少完成一次真实 `yaw_diff` 标定后版本的切枪验证。

## 13. 风险与注意事项
- 最大风险不是代码结构，而是几何关系没标定清楚。
- 若 USB 外参不准，切枪方向即使大体正确，也会在最终主工业相机接管时出现明显偏差。
- 若 `yaw_diff` 的方向或零点判断错误，会导致背向切枪朝错误方向转动。
- 若主相机与 USB 链路都直接参与控制且没有清晰状态机，系统会出现抢控制权和命令抖动。

## 14. 文档同步要求
- 若 `yaw_diff` 的单位、方向或零点被实测确认，立即同步：
  - `.agent/KNOWLEDGE.md`
  - `.agent/TROUBLESHOOTING.md`
  - `.agent/TODO.md`
- 若 `omnidirectional_mpc` 的入口名、配置名或状态机策略改动，立即同步本文件。
- 若最终决定 USB 相机不再维护完整 `tracker`，也必须回写本文件，避免文档和实现分叉。

---
