---

# 项目架构与核心知识库 (KNOWLEDGE.md)

## 1. 入口总览

### 1.1 当前主入口
| 入口 | 通信链路 | 主要功能 | 关键事实 |
| :--- | :--- | :--- | :--- |
| `src/standard_mpc.cpp` | `io::Gimbal` 串口 + `io::ROS2` | 自瞄 + 打符 + 裁判系统发布 + 导航下发 | 主线程采图/检测，规划线程 10ms/200ms 下发自瞄控制，ROS 线程 20ms 发布/透传；当前无 `imshow`，运行状态主要看 `logs/`、`records/`、`MvSdkLog/`。 |
| `src/auto_aim_debug_mpc.cpp` | `io::Gimbal` 串口 | 自瞄调试 | 固定 10ms 规划线程；输出 Plotter JSON；用 `bullet_count` 推断 `fired`。 |
| `src/auto_buff_debug_mpc.cpp` | `io::Gimbal` 串口 | 打符调试 | 直接发送 buff MPC 结果。 |
| `src/standard.cpp` | `io::CBoard` CAN | 单线程自瞄 | `Aimer` 直接出角度并经 CAN 下发；当前未使用 `Shooter` 开火门控。 |
| `src/mt_standard.cpp` | `io::CBoard` CAN | 多线程自瞄 + 打符 | `CommandGener` 线程做 `Aimer + Shooter + send`，循环约 2ms。 |

### 1.2 当前自瞄主链路
1. `io::Camera::read(img, t)` 采图并给出时间戳。
2. `io::Gimbal::q(t - 6ms)` 或 `io::CBoard::imu_at(t - 1ms)` 取对齐后的姿态四元数。
3. `Solver::set_R_gimbal2world(q)` 更新当前姿态旋转矩阵。
4. `YOLO::detect(img)` 输出装甲板列表。
5. `Tracker::track(...)` 管理目标状态机并更新 `Target` EKF。
6. `Planner::plan(...)` 或 `Aimer::aim(...)` 生成控制角度。
7. 通过 `gimbal.send(...)` 或 `cboard.send(...)` 下发给下位机。

## 2. Gimbal 串口协议

### 2.1 包类型
`io/gimbal/gimbal.hpp` 定义了 5 类 `packed` 结构，其中真正参与视觉-下位机收发的主要是前 3 个：

| 结构体 | 方向 | 包头 | 用途 | 包长 |
| :--- | :--- | :--- | :--- | :--- |
| `GimbalToVision` | 下位机 -> 视觉 | `0x5A 0x01` | 姿态、模式、云台状态、弹速、弹计数 | 64 字节 |
| `VisionToGimbal` | 视觉 -> 下位机 | `0x5A 0x01` | 自瞄控制包 | 64 字节 |
| `NavToGimbal` | 视觉 -> 下位机 | `0x5A 0x02` | 导航/哨兵状态下发 | 64 字节 |
| `RefereePackage1` | 下位机 -> 视觉 | `0x5A 0x02` | 裁判系统状态包 1 | 64 字节 |
| `RefereePackage2` | 下位机 -> 视觉 | `0x5A 0x03` | 裁判系统状态包 2 | 64 字节 |

### 2.2 上行包 `GimbalToVision`
定义见 `io/gimbal/gimbal.hpp`。字段顺序如下：

| 字段 | 类型 | 含义 | 视觉侧当前是否使用 |
| :--- | :--- | :--- | :--- |
| `head[2]` | `uint8_t[2]` | 固定 `0x5A 0x01` | 是 |
| `DWT_stamp` | `float` | 下位机 DWT 时间戳，单位微秒 | 否 |
| `mode` | `uint8_t` | 0 空闲，1 自瞄，2 小符，3 大符 | 是 |
| `q[4]` | `float[4]` | 四元数，`wxyz` 顺序 | 是 |
| `pitch` | `float` | 下位机反馈俯仰角 | 是 |
| `pitch_vel` | `float` | 下位机反馈俯仰角速度 | 是 |
| `yaw` | `float` | 下位机反馈偏航角 | 是 |
| `yaw_vel` | `float` | 下位机反馈偏航角速度 | 是 |
| `yaw_diff` | `float` | 协议保留/差值字段 | 否 |
| `bullet_speed` | `float` | 弹速 | 是 |
| `bullet_count` | `uint16_t` | 累计发弹计数 | 是 |
| `reserved[12]` | `uint8_t[12]` | 保留 | 否 |
| `tail` | `uint8_t` | 固定 `0x55` | 是 |
| `check_sum` | `uint16_t` | 16 位和校验 | 是 |

### 2.3 下行包 `VisionToGimbal`

| 字段 | 类型 | 含义 |
| :--- | :--- | :--- |
| `head[2]` | `uint8_t[2]` | 固定 `0x5A 0x01` |
| `time_stamp` | `uint64_t` | 视觉侧时间戳；便捷接口里用 `steady_clock` 毫秒值自动填充 |
| `mode` | `uint8_t` | 0 不控制，1 控制但不开火，2 控制且开火 |
| `pitch` | `float` | 目标俯仰角 |
| `pitch_vel` | `float` | 目标俯仰角速度 |
| `pitch_acc` | `float` | 目标俯仰角加速度 |
| `yaw` | `float` | 目标偏航角 |
| `yaw_vel` | `float` | 目标偏航角速度 |
| `yaw_acc` | `float` | 目标偏航角加速度 |
| `reserved[26]` | `uint8_t[26]` | 保留 |
| `check_sum` | `uint16_t` | 16 位和校验 |
| `tail` | `uint8_t` | 固定 `0x55` |

### 2.4 下行包 `NavToGimbal`

| 字段 | 类型 | 含义 |
| :--- | :--- | :--- |
| `head[2]` | `uint8_t[2]` | 固定 `0x5A 0x02` |
| `time_stamp` | `uint64_t` | 视觉侧时间戳 |
| `chassis_status` | `uint8_t` | 底盘状态 |
| `sentry_status` | `uint8_t` | 哨兵状态 |
| `mode` | `uint8_t` | 0 对装甲板，1 对前哨站，2 对能量机关，3 对基地 |
| `vx` | `float` | 目标/导航速度 X |
| `vy` | `float` | 目标/导航速度 Y |
| `vyaw` | `float` | 当前便捷接口固定写 `0.0f` |
| `reserved[36]` | `uint8_t[36]` | 保留 |
| `check_sum` | `uint16_t` | 16 位和校验 |
| `tail` | `uint8_t` | 固定 `0x55` |

### 2.5 校验和与尾字节顺序

#### 2.5.1 上行包
- `GimbalToVision`、`RefereePackage1`、`RefereePackage2` 的布局都是 `... tail, check_sum`。
- `read_thread()` 先检查 `tail == 0x55`，再调用 `tools::verify_check_sum16(...)`。
- `verify_check_sum16()` 的算法是：
  - 把前 `dwLength - 2` 个字节逐字节累加成 `uint16_t`
  - 再与最后两个字节拼成的小端 `uint16_t` 比较

#### 2.5.2 下行包
- `VisionToGimbal`、`NavToGimbal` 的布局是 `... check_sum, tail`。
- `tools::append_check_sum()` 会把校验和写入倒数第 3、倒数第 2 个字节，最后一个字节保留给 `tail`。
- 这不是笔误，而是当前协议有意为之：代码注释说明“电控校验需要先校验 tail，因此校验码放在倒数第二、第三个字节”。

### 2.6 `io::Gimbal` 读线程的真实行为
- 构造函数只从 YAML 读取 `com_port`，调用 `serial_.setPort(com_port)` 和 `serial_.open()`；当前代码没有在 `gimbal.cpp` 里显式设置波特率、奇偶校验、停止位或 timeout。
- `read_thread()` 先读 2 字节包头，再根据第二字节分支：
  - `0x01` 读 `GimbalToVision`
  - `0x02` 读 `RefereePackage1`
  - `0x03` 读 `RefereePackage2`
- 当 `0x01` 包校验通过后：
  - 使用主机侧 `std::chrono::steady_clock::now()` 作为接收时间
  - 把四元数压入 `queue_`
  - 更新 `state_`
  - 把 `mode` 映射成 `GimbalMode`
- 当前视觉侧没有使用上行里的 `DWT_stamp` 做时间同步。
- 连续读异常超过 2 秒会触发重连，最多尝试 10 次。

### 2.7 `q(t)` 的语义
- `Gimbal::q(t)` 不是“取最近一帧”，而是从四元数队列中取相邻两帧做 `slerp`。
- `src/standard_mpc.cpp` 与 `src/auto_aim_debug_mpc.cpp` 当前都按 `t - 6ms` 取姿态。
- 这意味着图像和姿态对齐问题，优先从这个固定补偿量排查。

## 3. 控制语义

### 3.1 `gimbal.state()`
`state()` 只返回 6 个量：
- `yaw`
- `yaw_vel`
- `pitch`
- `pitch_vel`
- `bullet_speed`
- `bullet_count`

当前上层用法：
- `Planner::plan(...)` 用 `bullet_speed`
- `auto_aim_debug_mpc.cpp` 用 `bullet_count` 递增判断 `fired`
- Plotter 直接记录 `yaw/pitch` 与角速度

### 3.2 `gimbal.send(...)`
便捷接口 `gimbal.send(control, fire, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc)` 做两件事：
1. 根据 `control/fire` 把 `mode` 编码成 0/1/2
2. 自动填当前 `steady_clock` 毫秒时间戳并计算校验和

### 3.3 `yaw` / `pitch` 的来源
- MPC 路径：`Planner::aim()` 计算
  - `yaw = atan2(y, x) + yaw_offset`
  - `pitch = -bullet_traj.pitch - pitch_offset`
- 旧 Aimer 路径：`Aimer::aim()` 也使用同样的 `yaw` / `pitch` 公式

### 3.4 `pitch` 的符号约定
- `tools::Trajectory::pitch` 的定义是“抬头为正”。
- 但发送给 `gimbal.send(...)` 的控制量使用了负号：
```cpp
pitch = -bullet_traj.pitch - pitch_offset;
```
- 当前文档必须区分这两层含义：
  - 几何/弹道解算里的 `pitch`：抬头为正
  - 下发给下位机的控制 `pitch`：当前接口约定下，抬头方向为负

## 4. 坐标与姿态链路

### 4.1 视觉侧如何使用四元数
1. 上行包提供 `q[4]`，`wxyz` 顺序。
2. `gimbal.q(t)` 对四元数按时间做 `slerp`。
3. `Solver::set_R_gimbal2world(q)` 计算当前 `R_gimbal2world_`。
4. `Solver::solve()` 把装甲板从相机系转到云台系，再转到世界系。

### 4.2 `R_gimbal2world_` 的实际公式
```cpp
R_gimbal2world_ =
  R_gimbal2imubody_.transpose() * q.toRotationMatrix() * R_gimbal2imubody_;
```

### 4.3 主链路中的时间对齐
- `standard_mpc`：`q = gimbal.q(t - 6ms)`
- `auto_aim_debug_mpc`：`q = gimbal.q(t - 6ms)`
- `auto_buff_debug_mpc`：当前直接 `q = gimbal.q(t)`，没有减 6ms

## 5. 上层如何消费 gimbal 协议

### 5.1 `standard_mpc.cpp`
- 主线程：
  - 读图
  - 取 `gimbal.mode()`
  - 取 `gimbal.q(t - 6ms)` 与 `gimbal.state()`
  - 自瞄模式下检测/跟踪并把目标放入队列
  - 打符模式下直接算 `buff_plan` 并 `gimbal.send(...)`
- 规划线程：
  - 自瞄模式下取 `target_queue.front()`
  - `planner.plan(target, gs.bullet_speed)`
  - `gimbal.send(...)`
  - 有目标时 `10ms`，否则 `200ms`
- ROS 线程：
  - 发布裁判系统数据
  - `gimbal.send(0, chassis_status, sentry_status, vx, vy)`
  - 固定 `20ms`
- 当前没有 `cv::imshow(...)`，属于无界面主程序。
- 运行中通常会新建：
  - `logs/<timestamp>.log`
  - `records/<timestamp>.txt`
  - `records/<timestamp>.avi`
  - `MvSdkLog/CamCtrl_00.log`
  - `MvSdkLog/MvUsb3vTL_00.log`

### 5.2 `auto_aim_debug_mpc.cpp`
- 固定 10ms 规划线程
- 使用 `bullet_count` 增量定义 `fired`
- Plotter 记录：
  - `gimbal_yaw`
  - `gimbal_yaw_vel`
  - `gimbal_pitch`
  - `gimbal_pitch_vel`
  - `target_yaw`
  - `target_pitch`
  - `plan_yaw`
  - `plan_pitch`
  - `fire`
  - `fired`

## 6. Tracker / Planner 的当前关键事实

### 6.1 Tracker
- 先按 `enemy_color` 过滤装甲板，再按图像中心距离和优先级排序。
- 状态机：`lost -> detecting -> tracking -> temp_lost`，全向感知分支还有 `switching`。
- EKF 状态向量固定为 11 维：
```text
[x, vx, y, vy, z, vz, a, w, r, l, h]
```

### 6.2 Planner
- `DT = 0.01`，`HORIZON = 100`。
- yaw 和 pitch 分别单独求 TinyMPC。
- `plan(std::optional<Target>, bullet_speed)` 先按目标角速度选择高/低速延迟，再预测到未来。
- `plan(Target, bullet_speed)` 再按弹道飞行时间继续预测。
- `fire` 的判据是规划参考轨迹和 MPC 状态轨迹在 `shoot_offset = 2` 处的误差是否小于 `fire_thresh`。

## 7. 维护入口
- 改 `gimbal` 协议时：优先改 `.agent/KNOWLEDGE.md` 第 2、3、4、5 节。
- 改串口校验或尾字节顺序时：同时改 `.agent/TROUBLESHOOTING.md`。
- 改 `q(...)` 的时间补偿时：同时改 `.agent/TODO.md` 和 `.agent/TROUBLESHOOTING.md`。

---
