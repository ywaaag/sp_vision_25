---
#  项目架构与核心知识库 (KNOWLEDGE.md) - 优化重组版

> **重要说明**：本文档已按逻辑主题重新组织，内容完整性与原版一致，仅结构调整以提升可读性。

---

## 第一部分：系统概述

### 1.1 全局数据流 (System Pipeline)
AI 在修改代码前必须理解数据是如何从相机流向云台的：
1. **Producer**: `io/camera` 捕获图像，附加硬时间戳。
2. **Detection**: `tasks/auto_aim/detector` 输出 `Armor` 候选框序列（图像坐标系）。
3. **PnP Solver**: `tasks/auto_aim/solver` 将图像坐标转为相对于相机的 3D 坐标。
4. **Coordinate Transform**: 将相机坐标转为全局/机器人惯性坐标系（结合 `io/gimbal` 反馈的四元数）。
5. **Tracking**: `tasks/auto_aim/tracker` 喂入 3D 点，使用 **EKF** 预测目标下一帧位姿。
6. **Decision**: `tasks/auto_aim/aimer` 根据预测位姿和弹道模型计算发射偏置。
7. **Control**: `tasks/auto_aim/planner` 使用 **MPC** 生成平滑云台轨迹。
8. **Consumer**: `io/cboard` 通过 CAN 总线发送控制指令。

### 1.2 分层架构与接口协议
| 层次 | 核心目录 | AI 准则 | 关键类/接口 |
| :--- | :--- | :--- | :--- |
| **L1: 硬件** | `io/` | 严禁在算法层直接调用 SDK。 | `CameraBase`, `CBoard`, `Gimbal` |
| **L2: 工具** | `tools/` | 必须保持无状态，线程安全。 | `ThreadSafeQueue`, `Logger`, `EKF` |
| **L3: 任务** | `tasks/` | 模块间通过结构体（如 `TargetInfo`）传递。 | `Detector`, `Tracker`, `Solver`, `Aimer` |
| **L4: 应用** | `src/` | 负责实例化各层并建立 Data Pipeline。 | `standard_mpc.cpp`（标准自瞄+打符）, `auto_aim_debug_mpc.cpp`（调试自瞄） |
| **L5: 配置** | `configs/` | 所有硬编码参数是 Bug，必须读 YAML。 | `ConfigManager` |

### 1.3 核心概念速览
- **坐标系系统**：7 个坐标系（像素 → 相机 → 云台 → 世界）的精确变换链（§2）。
- **通信协议**：串口（云台控制）与 CAN 总线（IMU 四元数）双向通信（§3）。
- **视觉处理**：YOLO 检测 → 多目标跟踪 → EKF 状态估计（§4）。
- **轨迹规划**：MPC 生成平滑云台轨迹，弹道补偿重力与空气阻力（§5）。
- **多线程模型**：规划线程（10ms/200ms）、检测线程、通信线程协同（§6）。
- **调试机制**：JSON 数据记录、重投影可视化、离线回放（§7）。
- **特殊模式**：打符（Buff）模式独立检测与规划（§8）。

---

## 第二部分：坐标系与变换（核心基础）

**（最高优先级）** 在修改任何坐标相关代码前，必须彻底理解以下 7 个坐标系及其变换链。坐标系混乱是项目中最常见的 Bug 来源。

### 2.1 坐标系定义（从底层到顶层）

| 坐标系 | 别名 | 原点 | 轴定义（右手系） | 关键变量/类 | 备注 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **1. 图像像素坐标系** | `Pixel` | 图像左上角 | $u$ 向右，$v$ 向下（像素单位） | `armor.points`（4个`cv::Point2f`） | OpenCV 标准，与相机内参关联。 |
| **2. 图像归一化坐标系** | `Norm` | 图像左上角 | $x = u/W$, $y = v/H$，范围 $[0,1]$ | `armor.center_norm` (`cv::Point2f`) | 用于视场角转换：$\Delta\theta = \frac{FOV}{2} - x \times FOV$。 |
| **3. 相机坐标系** | `Camera` | 相机光心 | $X_c$ 向右，$Y_c$ 向下，$Z_c$ 向前（光轴） | `xyz_in_camera`（`Eigen::Vector3d`） | **OpenCV 标准**：Z 向前，Y 向下，X 向右。PnP 解算的输出坐标系。 |
| **4. 云台坐标系** | `Gimbal` | 云台旋转中心 | $X_g$ 向前（相机光轴），$Y_g$ 向左，$Z_g$ 向上 | `armor.xyz_in_gimbal`, `armor.ypr_in_gimbal` | **控制基准**：所有云台角度（yaw/pitch）均以此系为参考。 |
| **5. IMU 机体坐标系** | `IMUBody` | IMU 传感器中心（≈机器人底盘中心） | 由标定定义，通常与云台系对齐 | `R_gimbal2imubody_`（固定矩阵） | 下位机四元数 $q$ 的参考系。配置文件默认为单位矩阵。 |
| **6. 世界坐标系** | `World` | **机器人当前位置**（地面投影点） | $X_w$ 向前（机器人初始朝向），$Y_w$ 向左，$Z_w$ 垂直向上 | `armor.xyz_in_world`, `target->ekf_x()[0..2]` | **全局参考系**：所有目标状态估计必须在此系下进行，避免云台自运动干扰。 |
| **7. 装甲板模型坐标系** | `Armor` | 装甲板几何中心 | $X_a$ 垂直装甲板向外，$Y_a$ 沿宽度方向向右，$Z_a$ 沿高度方向向上 | `BIG_ARMOR_POINTS`, `SMALL_ARMOR_POINTS` | 固定 3D 模型，用于 PnP 解算。 |

### 2.2 关键变换矩阵（存储于 `Solver` 类）

| 矩阵 | 维度 | 含义 | 配置键 | 备注 |
| :--- | :--- | :--- | :--- | :--- |
| `camera_matrix_` | 3×3 | 相机内参（焦距、主点） | `camera_matrix` | 将相机坐标系下的 3D 点投影到像素坐标。 |
| `distort_coeffs_` | 1×5 | 镜头畸变系数（径向+切向） | `distort_coeffs` | 用于去畸变和精确重投影。 |
| `R_camera2gimbal_` | 3×3 | **相机 → 云台**的旋转矩阵 | `R_camera2gimbal` | **手眼标定结果**。描述相机相对于云台的安装角度。 |
| `t_camera2gimbal_` | 3×1 | **相机 → 云台**的平移向量（单位：m） | `t_camera2gimbal` | 相机光心到云台旋转中心的偏移。 |
| `R_gimbal2imubody_` | 3×3 | **云台 → IMU 机体**的旋转矩阵 | `R_gimbal2imubody` | 默认为单位矩阵（云台与 IMU 对齐）。 |
| `R_gimbal2world_` | 3×3 | **云台 → 世界**的旋转矩阵（动态） | – | 由 `set_R_gimbal2world(q)` 实时计算，依赖云台四元数 $q$。 |

### 2.3 变换链公式（必须死记）

**完整正向变换（像素 → 世界）：**
1. **去畸变**（可选）：`cv::undistortPoints()` 将像素坐标校正为理想针孔投影。
2. **PnP 解算**：已知装甲板 4 个角点在**装甲板模型坐标系**下的坐标 $P_a$，通过 `cv::solvePnP()` 求解**相机坐标系**下的位姿 $(R_{a2c}, t_{a2c})$。
3. **相机 → 云台**：
   $$
   P_g = R_{c2g} \cdot P_c + t_{c2g}, \quad R_{a2g} = R_{c2g} \cdot R_{a2c}
   $$
   其中 $P_c = t_{a2c}$（装甲板中心在相机系下的坐标）。
4. **云台 → 世界**：
   $$
   P_w = R_{g2w} \cdot P_g, \quad R_{a2w} = R_{g2w} \cdot R_{a2g}
   $$
   其中 $R_{g2w}$ 由 §2.4 实时计算。

**逆向变换（世界 → 像素）：**
- 用于重投影可视化：`solver.reproject_armor(xyz_in_world, yaw, type, name)`。
- 先将世界坐标 $P_w$ 转换回云台系 $P_g = R_{g2w}^{-1} P_w = R_{g2w}^T P_w$（旋转矩阵正交）。
- 再转换到相机系 $P_c = R_{c2g}^T (P_g - t_{c2g})$。
- 最后通过 `cv::projectPoints()` 投影到像素坐标（考虑畸变）。

### 2.4 云台四元数 $q$ 与世界坐标系建立

**下位机协议**：每 1ms 发送 `GimbalToVision` 数据包，包含四元数 $q$（wxyz 顺序）、云台模式、角度、角速度、弹速、弹计数。

**$q$ 的物理意义**：表示 **IMU 机体坐标系** 到 **世界坐标系** 的旋转。
- 世界坐标系定义为：机器人**当前位置为原点**，**地面为 XY 平面**，**Z 轴垂直向上**的右手系。
- IMU 机体坐标系通常与机器人底盘固连，由标定确定。

**实时计算 $R_{g2w}$**（`solver.set_R_gimbal2world(q)`）：
```cpp
Eigen::Matrix3d R_imubody2world = q.toRotationMatrix();  // q: IMUBody → World
R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2world * R_gimbal2imubody_;
```
- 若 `R_gimbal2imubody_ = I`（单位矩阵），则 `R_gimbal2world_ = R_imubody2world`。
- 该矩阵将云台系下的向量旋转到世界系：`P_w = R_gimbal2world_ * P_g`。

**时间补偿**：图像时间戳 $t_{img}$ 与四元数时间戳存在固定延迟（曝光、传输）。调用 `gimbal.q(t_img - 6ms)` 进行线性插值，获取精确对应时刻的姿态。

### 2.5 云台角度定义（控制接口）

**角度基准**：所有云台角度（`yaw`, `pitch`）均在 **世界坐标系** 下定义（**关键修正**）。
- **偏航角 yaw**：世界坐标系下，目标相对于机器人当前位置的方位角（绕 $Z_w$ 轴旋转）。$0$ 度对应世界坐标系 $X_w$ 轴正向（机器人初始朝向），**正方向**为逆时针（向左转为正）。
- **俯仰角 pitch**：世界坐标系下，目标相对于机器人当前位置的俯仰角（绕 $Y_w$ 轴旋转）。$0$ 度对应水平面，**正方向**为抬头（向上转）。

**控制指令坐标系澄清**：
- **历史误解**：以往认为 `gimbal.send()` 发送的是云台坐标系下的绝对角度或增量。
- **实际真相**：发送的 `yaw`、`pitch` 是 **世界坐标系下的绝对角度**，由 `Planner::plan()` 通过 `aim()` 函数计算得出。
- **计算过程**：
  1. `aim()` 函数接收目标在世界坐标系下的位置 `xyz`（来自 `Target::armor_xyza_list()`）。
  2. 计算方位角：`azim = std::atan2(xyz.y(), xyz.x())`（世界坐标系 XY 平面内，从 $X_w$ 轴到目标向量的角度）。
  3. 加上固定偏移：`yaw = azim + yaw_offset_`，`pitch = -bullet_traj.pitch - pitch_offset_`（弹道补偿后的俯仰角）。
  4. MPC 围绕该目标角度生成平滑轨迹，最终 `plan.yaw`、`plan.pitch` 即为世界坐标系下的目标角度。
- **下位机职责**：下位机接收世界坐标系下的目标角度，结合当前云台四元数 $q$（表示 IMU 机体坐标系到世界坐标系的旋转）与云台编码器反馈 `gs.yaw`、`gs.pitch`（云台相对于底盘的角度），**内部计算所需的云台控制量**。视觉端无需关心云台当前姿态，实现了解耦。

**为什么这是正确的**：
1. **解耦设计**：视觉模块仅负责在世界坐标系下计算目标角度，无需感知云台实时姿态。
2. **下位机全状态反馈**：下位机每 1ms 发送四元数 $q$、云台角度 `yaw`、`pitch`，具备完整的坐标系转换信息。
3. **实时性保障**：坐标转换与闭环控制在下位机执行，延迟更低，避免视觉计算延迟带来的控制误差。

**控制指令**：`gimbal.send(control, fire, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc)` 发送给下位机，下位机负责执行位置-速度-加速度三闭环控制（在世界坐标系下）。

### 2.6 相机-枪口标定与弹道补偿

**问题**：相机与云台（枪口）存在物理偏移，相机观测的目标位置 ≠ 子弹发射线指向。如何保证子弹命中？

**解决方案**：通过手眼标定矩阵 `R_camera2gimbal_`、`t_camera2gimbal_` 与弹道补偿模型。

**1. 手眼标定（相机 → 云台）**
- **标定矩阵**：`R_camera2gimbal_`（旋转）、`t_camera2gimbal_`（平移）在 `configs/standard3.yaml` 中配置。
- **作用**：将相机坐标系下的目标位置 $P_c$ 转换到云台坐标系 $P_g$：
  $$
  P_g = R_{c2g} \cdot P_c + t_{c2g}
  $$
- **物理意义**：补偿相机与云台旋转中心之间的安装偏移（约几厘米）。即使相机安装在云台侧面或上方，仍能准确计算目标相对于枪口的位置。

**2. 弹道补偿（重力与空气阻力）**
- **子弹飞行时间（TOF）**：`tools::Trajectory` 类迭代求解子弹到达目标所需时间 $t_f$，考虑空气阻力 $F = -kv^2$。
- **重力下垂**：子弹在飞行期间受重力下落，俯仰角需提前抬升 `bullet_traj.pitch`。
- **补偿计算**：在 `Planner::aim()` 中：
  ```cpp
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  return {tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_};
  ```
  - `yaw_offset_`、`pitch_offset_` 为机械零位校准偏移。
  - **负号**：子弹下落需向上抬升，故俯仰角取负值。

**3. 坐标系整合**
1. 相机观测装甲板角点（像素坐标）→ 通过 PnP 得到相机坐标系下的位置 $P_c$。
2. 手眼标定：$P_g = R_{c2g} P_c + t_{c2g}$，得到目标相对于云台旋转中心的位置。
3. 云台 → 世界：$P_w = R_{g2w} P_g$，转换到世界坐标系（用于 EKF 滤波）。
4. 弹道补偿：根据 $P_w$ 计算世界坐标系下的目标方位角 `azim` 与俯仰补偿 `pitch`。
5. 发送世界坐标系下的目标角度 `(yaw, pitch)` 给下位机。

**4. 运动补偿**
- **云台自身运动**：下位机通过四元数 $q$ 实时感知云台在世界坐标系下的姿态，将视觉发送的世界坐标系目标角度转换为云台电机控制量。
- **目标运动**：EKF 预测目标在 $t_f$ 时间后的位置，实现运动补偿。

**总结**：相机-枪口标定确保了几何对准，弹道补偿解决了物理下落，坐标系整合实现了模块解耦，运动补偿处理了动态误差。四者结合保证子弹命中。

### 2.7 常用坐标获取方式（代码索引）

| 坐标 | 获取方式 | 数据结构 | 单位 |
| :--- | :--- | :--- | :--- |
| 装甲板图像角点 | `armor.points` | `std::vector<cv::Point2f>` | 像素 |
| 装甲板中心（归一化）| `armor.center_norm` | `cv::Point2f` | 无量纲 [0,1] |
| 装甲板在相机系坐标 | `xyz_in_camera`（临时变量） | `Eigen::Vector3d` | 米 |
| 装甲板在云台系坐标 | `armor.xyz_in_gimbal` | `Eigen::Vector3d` | 米 |
| 装甲板在世界系坐标 | `armor.xyz_in_world` | `Eigen::Vector3d` | 米 |
| 装甲板在世界系偏航角 | `armor.ypr_in_world[0]` | `double` | 弧度 |
| 目标旋转中心在世界系坐标 | `target->ekf_x()[0]`（x）, `[2]`（y）, `[4]`（z） | `double` | 米 |

### 2.8 易错点与验证方法

1. **坐标系混淆**：`armor.xyz_in_world` 是**装甲板中心**，`target->ekf_x()[0..2]` 是**目标旋转中心**，两者相差半径 $r$。
2. **旋转矩阵方向**：`R_camera2gimbal_` 是“相机 → 云台”，用于右乘：`P_g = R_camera2gimbal_ * P_c + t_camera2gimbal_`。
3. **四元数顺序**：下位机发送 `float q[4]` 为 **wxyz** 顺序，`Eigen::Quaterniond` 构造时为 `(w, x, y, z)`。
4. **角度单位**：代码内部统一使用**弧度**，仅输出显示时乘 $57.3$ 转为度。
5. **验证方法**：运行 `auto_aim_debug_mpc`，观察重投影点（红色）是否准确覆盖装甲板角点。若偏差大，检查标定矩阵与坐标系定义。

---


## 第三部分：通信协议与接口

### 3.1 下位机通信协议与数据接口

本项目通过两种物理链路与下位机通信：**串口**（云台控制与状态反馈）与 **CAN 总线**（IMU 四元数、弹速、模式等）。所有通信协议均定义在 `io/` 目录下，视觉端作为**客户端**，下位机作为**服务器**。

#### 3.1.1 串口通信（云台控制）

**硬件接口**：`/dev/gimbal`（串口，波特率通常为 115200）
**数据流向**：双向，每 1ms 周期同步。

##### 下行数据：`GimbalToVision`（下位机 → 视觉）
- **协议头**：`'C'` `'B'`（0x43 0x42）
- **数据包结构**（定义于 `io/gimbal/gimbal.hpp`）：
```cpp
struct __attribute__((packed)) GimbalToVision {
  uint8_t head[2] = {'C', 'B'};
  uint8_t mode;           // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];             // wxyz 顺序的四元数，表示 IMU 机体坐标系 → 世界坐标系的旋转
  float yaw;              // 云台当前偏航角（单位：rad）
  float yaw_vel;          // 云台当前偏航角速度（单位：rad/s）
  float pitch;            // 云台当前俯仰角（单位：rad）
  float pitch_vel;        // 云台当前俯仰角速度（单位：rad/s）
  float bullet_speed;     // 弹速（单位：m/s）
  uint16_t bullet_count;  // 子弹累计发送次数（用于开火检测）
  uint8_t tail = 0xff;
};
```
- **字段详解**：
  | 字段 | 物理意义 | 坐标系 | 单位 | 备注 |
  | :--- | :--- | :--- | :--- | :--- |
  | `mode` | 下位机当前工作模式 | – | 枚举 | 决定视觉端是否启动自瞄/打符逻辑 |
  | `q[4]` | **IMU 机体 → 世界**的旋转四元数 | 四元数（wxyz） | 无量纲 | **关键**：用于建立世界坐标系（见 §2.4） |
  | `yaw` | 云台当前偏航角 | 云台坐标系（相对于底盘） | **rad** | 从云台编码器读取，用于状态监控 |
  | `yaw_vel` | 云台偏航角速度 | 云台坐标系 | **rad/s** | 编码器差分或陀螺仪测量 |
  | `pitch` | 云台当前俯仰角 | 云台坐标系（相对于底盘） | **rad** | 同上 |
  | `pitch_vel` | 云台俯仰角速度 | 云台坐标系 | **rad/s** | 同上 |
  | `bullet_speed` | 弹丸初速度 | – | **m/s** | 由弹速测量模块提供，用于弹道补偿 |
  | `bullet_count` | 累计发射弹丸数 | – | 计数 | 通过比较相邻帧的增量判断实际击发（`fired`） |

##### 上行数据：`VisionToGimbal`（视觉 → 下位机）
- **协议头**：`'C'` `'B'`
- **数据包结构**：
```cpp
struct __attribute__((packed)) VisionToGimbal {
  uint8_t head[2] = {'C', 'B'};
  uint8_t mode;      // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;         // 目标偏航角（单位：rad）
  float yaw_vel;     // 目标偏航角速度（单位：rad/s）
  float yaw_acc;     // 目标偏航角加速度（单位：rad/s²）
  float pitch;       // 目标俯仰角（单位：rad）
  float pitch_vel;   // 目标俯仰角速度（单位：rad/s）
  float pitch_acc;   // 目标俯仰角加速度（单位：rad/s²）
  uint8_t tail = 0xff;
};
```
- **字段详解**：
  | 字段 | 物理意义 | 坐标系 | 单位 | 备注 |
  | :--- | :--- | :--- | :--- | :--- |
  | `mode` | 视觉控制指令 | – | 枚举 | `0`=释放控制权，`1`=仅控制，`2`=控制+开火 |
  | `yaw` | **目标偏航角**（绝对角度） | **世界坐标系** | **rad** | **关键**：世界坐标系下的绝对偏航角（绕 $Z_w$ 轴），$0$ 对应 $X_w$ 轴正向，**逆时针为正** |
  | `yaw_vel` | 目标偏航角速度 | 世界坐标系 | **rad/s** | MPC 输出的跟踪角速度 |
  | `yaw_acc` | 目标偏航角加速度 | 世界坐标系 | **rad/s²** | MPC 输出的跟踪角加速度 |
  | `pitch` | **目标俯仰角**（绝对角度） | **世界坐标系** | **rad** | 世界坐标系下的绝对俯仰角（绕 $Y_w$ 轴），$0$ 对应水平面，**抬头为正** |
  | `pitch_vel` | 目标俯仰角速度 | 世界坐标系 | **rad/s** | MPC 输出的跟踪角速度 |
  | `pitch_acc` | 目标俯仰角加速度 | 世界坐标系 | **rad/s²** | MPC 输出的跟踪角加速度 |

**重要澄清**：视觉端发送的 `yaw`/`pitch` 是 **世界坐标系下的绝对角度**，而非增量或云台坐标系下的角度。下位机内部根据当前四元数 $q$（IMU→World）和云台编码器反馈，自行计算所需的云台控制量，实现**姿态解耦**。

#### 3.1.2 CAN 总线通信（CBoard）

**硬件接口**：`can0`（SocketCAN）
**数据流向**：双向，CAN 帧周期通常为 1ms。

##### 下行数据（下位机 → 视觉）
- **四元数帧**（`quaternion_canid`，默认为 `0x100`）：
  - **数据格式**：8 字节，每 2 字节为一个有符号整型，依次为 $x, y, z, w$（**xyzw 顺序**），实际值为 `int16_t / 1e4`。
  - **物理意义**：与串口四元数 `q[4]` 相同，表示 **IMU 机体坐标系 → 世界坐标系** 的旋转。
  - **单位**：无量纲（四元数已归一化）。
- **弹速与模式帧**（`bullet_speed_canid`，默认为 `0x101`）：
  - **数据格式**：8 字节，结构为：
    ```cpp
    int16_t bullet_speed;  // 单位：0.01 m/s（实际值 = int16_t / 1e2）
    uint8_t mode;          // 0: idle, 1: auto_aim, 2: small_buff, 3: big_buff, 4: outpost
    uint8_t shoot_mode;    // 0: left_shoot, 1: right_shoot, 2: both_shoot（哨兵专有）
    int16_t ft_angle;      // 无人机专有，单位：0.0001 rad（实际值 = int16_t / 1e4）
    ```
  - **物理意义**：提供弹速、当前模式、射击模式（哨兵）等附加信息。

##### 上行数据（视觉 → 下位机）
- **控制指令帧**（`send_canid`，默认为 `0xff`）：
  - **数据格式**：8 字节，结构为：
    ```cpp
    uint8_t control;      // 1=控制，0=释放
    uint8_t shoot;        // 1=开火，0=不开火
    int16_t yaw;          // 单位：0.0001 rad（实际值 = int16_t / 1e4）
    int16_t pitch;        // 单位：0.0001 rad（实际值 = int16_t / 1e4）
    int16_t horizon_distance; // 无人机专有，单位：0.0001 m（实际值 = int16_t / 1e4）
    ```
  - **物理意义**：通过 CAN 总线发送的控制指令，与串口 `VisionToGimbal` 语义一致，但精度更高（0.0001 rad）。

### 3.2 理想下位机坐标系约定

本项目期望下位机遵循**右手坐标系**，定义如下：

| 坐标系 | 原点 | $X$ 轴 | $Y$ 轴 | $Z$ 轴 | 备注 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **IMU 机体坐标系** | IMU 传感器中心（≈机器人底盘中心） | **向前**（机器人前进方向） | **向左** | **向上** | 与机器人底盘固连，右手系（$X \times Y = Z$） |
| **世界坐标系** | 机器人当前位置（地面投影点） | **向前**（机器人初始朝向） | **向左** | **向上** | 地面为 $XY$ 平面，$Z$ 垂直向上，右手系 |

**四元数约定**：
- **顺序**：**wxyz**（串口）/ **xyzw**（CAN 总线，但视觉端会转换为 wxyz）。
- **变换方向**：四元数 $q$ 表示从 **IMU 机体坐标系** 到 **世界坐标系** 的旋转。
  - 即：$P_{\text{world}} = q \cdot P_{\text{IMUBody}} \cdot q^{-1}$（向量旋转）。
  - 在代码中通过 `Eigen::Quaterniond q(w, x, y, z)` 构造，`q.toRotationMatrix()` 得到 $R_{\text{IMUBody} \to \text{World}}$。

**欧拉角约定（RPY）**：
- **偏航角（Yaw）**：绕 $Z$ 轴旋转，$0$ 度对应 $X$ 轴正向，**逆时针为正**（向左转为正）。
- **俯仰角（Pitch）**：绕 $Y$ 轴旋转，$0$ 度对应水平面，**抬头为正**（向上转为正）。
- **滚转角（Roll）**：绕 $X$ 轴旋转，本项目云台无滚转自由度，通常为 $0$。

**控制指令语义**：
- 视觉端发送的 `yaw`/`pitch` 是在 **世界坐标系** 下定义的绝对角度。
- 下位机需根据当前四元数 $q$（IMU→World）和云台编码器反馈，计算云台需要转动的角度增量。
- **示例**：若世界坐标系下目标偏航角为 $\theta_{\text{world}}$，当前云台相对于底盘的偏航角为 $\theta_{\text{gimbal}}$，且四元数 $q$ 表示底盘相对于世界的旋转为 $R$，则下位机内部计算：
  \[
  \theta_{\text{command}} = \theta_{\text{world}} - \text{yaw}(R)
  \]
  其中 $\text{yaw}(R)$ 为旋转矩阵 $R$ 提取的偏航角（底盘相对于世界的偏航）。

### 3.3 时间同步与补偿

- **四元数插值**：下位机以 1ms 周期发送四元数，视觉端通过 `gimbal.q(t)` 或 `cboard.imu_at(t)` 进行线性插值（球面线性插值），获取图像时间戳 $t_{\text{img}}$ 对应的精确姿态。
- **固定延迟补偿**：图像采集存在曝光、传输等延迟，通常调用 `gimbal.q(t_img - 6ms)` 补偿 6ms。
- **实时性要求**：从图像采集到指令发送的端到端延迟必须 **< 33ms**，否则目标已运动，命中率下降。

---


## 第四部分：视觉处理流水线

### 4.1 标定知识
* **内参矩阵**: 存放于 `camera.yaml`。修改 `Solver` 逻辑前必须确认内参读取正确。
* **手眼坐标系 (Hand-Eye)**:
    * `T_camera_gimbal`: 相机坐标系到云台中心坐标系的转换矩阵。
    * **注意**: 不同的机器人（步兵 3 号 vs 4 号）该矩阵不同。
* **YOLO 推理**: 使用 OpenVINO 后端。模型输入通常为 $640 \times 640$ 或 $416 \times 416$。

### 4.2 YOLO 检测
```cpp
auto armors = yolo.detect(img);  // 返回 std::list<Armor>
```
- **模型后端**：OpenVINO，输入分辨率 640×640 或 416×416。
- **输出**：每个装甲板的类别（颜色+数字）、置信度、边界框、4 个角点图像坐标。
- **Armor 构造**：根据检测结果初始化 `Armor` 对象，包括 `color`, `type`, `name`, `priority`, `points`（图像角点）。

### 4.3 多目标跟踪（Tracker）
```cpp
auto targets = tracker.track(armors, t);  // 返回 std::list<Target>
```
- **状态机**：`DETECTING` → `TRACKING` → `LOST`，根据连续检测次数与临时丢失次数切换。
- **目标关联**：当前帧装甲板与已有目标通过距离、角度、颜色、数字进行匈牙利匹配。
- **目标初始化**：匹配成功的装甲板调用 `Target::update(armor)` 更新 EKF；未匹配的装甲板新建 `Target` 对象。
- **EKF 预测**：每个目标在更新前先执行 `Target::predict(t)`，基于时间差 `dt` 进行状态预测。

### 4.4 扩展卡尔曼滤波 (EKF) 规格
* **状态向量 ($X$)**: $[x, v_x, y, v_y, z, v_z, a, \omega, r, l, h]^T$（共 11 维）。
  - $x, y, z$：目标旋转中心在世界坐标系下的位置（单位：m）。
  - $v_x, v_y, v_z$：目标在对应方向上的速度（单位：m/s）。
  - $a$：目标当前朝向角度（单位：rad）。
  - $\omega$：目标旋转角速度（单位：rad/s）。
  - $r$：目标半径（单位：m）。
  - $l$：$r_2 - r_1$（备用）。
  - $h$：$z_2 - z_1$（备用）。
* **坐标系**: 必须在 **世界坐标系（World Frame）** 下进行滤波，防止云台自身运动干扰目标运动估计。
* **协方差调整**: 观测噪声 $R$ 必须根据目标距离动态调整（距离越远，$R$ 越大）。
* **代码索引参考**：`target->ekf_x()[0]`=x, `[1]`=vx, `[2]`=y, `[3]`=vy, `[4]`=z, `[5]`=vz, `[6]`=a, `[7]`=ω, `[8]`=r, `[9]`=l, `[10]`=h。

#### 4.4.1 观测模型
- **观测向量**：$Z = [x_a, y_a, z_a, a]^T$，即装甲板在世界坐标系下的位置与朝向角。
- **观测矩阵 $H$**：由 `Target::h_jacobian` 计算，将旋转中心状态映射到具体装甲板位置。
- **观测噪声 $R$**：根据目标距离动态调整，距离越远噪声越大。

#### 4.4.2 状态转移
- **状态转移矩阵 $F$**：线性匀速模型，对角块结构，位置、速度、角度、角速度分别积分。
- **过程噪声 $Q$**：分段白噪声模型，根据目标类型（前哨站/普通机器人）设置不同的加速度方差。

---
## 第五部分：轨迹规划与控制

### 5.1 MPC规划原理与弹道补偿
* **空气阻力模型**: 采用迭代法计算子弹飞行时间（TOF），考虑空气阻力系数 $k$。
* **TinyMPC 约束**:
    * 状态约束：云台角度范围（Pitch/Yaw Limit）。
    * 输入约束：云台最大角速度、最大角加速度。
* **重力补偿**: 必须基于目标实时距离进行位姿抬升计算。

### 5.2 规划线程调度
```cpp
std::thread plan_thread([&]() {
    while (!quit) {
        if (!target_queue.empty() && mode == io::GimbalMode::AUTO_AIM) {
            auto target = target_queue.front();
            auto gs = gimbal.state();
            auto plan = planner.plan(target, gs.bullet_speed);
            // 发送指令
            std::this_thread::sleep_for(10ms);
        } else
            std::this_thread::sleep_for(200ms);
    }
});
```
- **目标队列**：`ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1)` 容量为 1，最新目标覆盖旧值。
- **调度策略**：有目标且处于自瞄模式时以 **10ms** 周期运行；否则休眠 **200ms** 降低 CPU 占用。
- **线程安全**：`quit` 为 `std::atomic<bool>`，`mode` 为 `std::atomic<io::GimbalMode>`。

### 5.3 弹道补偿与瞄准点计算
```cpp
Eigen::Matrix<double, 2, 1> aim(const Target & target, double bullet_speed);
```
1. **子弹飞行时间（TOF）迭代**：考虑空气阻力 $F = -kv^2$，迭代求解子弹到达目标所需时间 $t_f$。
2. **目标预测**：使用 EKF 状态向量预测 $t_f$ 时间后的目标位置 $[x_p, y_p, z_p]^T$。
3. **重力补偿**：根据弹道下垂量计算 pitch 方向抬升量。
4. **坐标系转换**：将预测的世界坐标转换为云台坐标系下的偏航、俯仰角 $(yaw_{target}, pitch_{target})$。

### 5.4 TinyMPC 轨迹生成
- **状态向量**：$[yaw, yaw_{vel}, pitch, pitch_{vel}]^T$。
- **输入向量**：$[yaw_{acc}, pitch_{acc}]^T$。
- **约束**：
  - 状态约束：云台机械角度限位。
  - 输入约束：最大角加速度。
- **目标函数**：跟踪瞄准点 $(yaw_{target}, pitch_{target})$，同时最小化控制量变化。
- **求解器**：`TinySolver` 分别对 yaw 和 pitch 轴进行求解，返回 **1 秒** 的轨迹（100 个点，10ms 间隔）。

### 5.5 开火决策
```cpp
plan.fire = (std::abs(target->ekf_x()[7]) > decision_speed_)
            ? (error < fire_thresh_high_speed_)
            : (error < fire_thresh_low_speed_);
```
- **决策逻辑**：
  - 目标角速度 > `decision_speed_`：视为高速移动，使用较宽的火阈值 `fire_thresh_high_speed_`。
  - 否则：使用较严的火阈值 `fire_thresh_low_speed_`。
- **误差计算**：当前云台指向与瞄准点之间的角度差。

### 5.6 控制指令输出
#### 5.6.1 指令结构
```cpp
gimbal.send(plan.control, plan.fire,
            plan.yaw, plan.yaw_vel, plan.yaw_acc,
            plan.pitch, plan.pitch_vel, plan.pitch_acc);
```
- **control**：`true` 表示启用云台控制，`false` 仅停止指令。
- **fire**：`true` 请求开火，需要下位机检查弹舱状态与射速限制。
- **yaw, pitch**：目标角度（rad）。
- **yaw_vel, pitch_vel**：目标角速度（rad/s）。
- **yaw_acc, pitch_acc**：目标角加速度（rad/s²）。

#### 5.6.2 协议封装
```cpp
VisionToGimbal tx_data;
tx_data.mode = (control ? (fire ? 2 : 1) : 0);
tx_data.yaw = yaw; tx_data.yaw_vel = yaw_vel; tx_data.yaw_acc = yaw_acc;
tx_data.pitch = pitch; tx_data.pitch_vel = pitch_vel; tx_data.pitch_acc = pitch_acc;
```
- **模式字节**：0=空闲，1=控制但不开火，2=控制且开火。
- **串口发送**：通过 `serial::Serial` 以 1ms 周期发送，包头 `"CB"`，包尾 `0xFF`。

---
## 第六部分：多线程与实时性

### 6.1 多线程架构
* **线程优先级**: 控制线程（规划）> 串口/CAN 线程 > 图像采集 > 视觉处理（检测+跟踪）。
* **同步机制**:
    * **线程间通信**: 使用 `tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true>`（容量为 1）实现检测线程与规划线程之间的松耦合数据传递。
    * **模式切换**: 使用 `std::atomic<io::GimbalMode>` 实时读取云台模式，确保控制逻辑与硬件状态同步。
    * **退出标志**: 使用 `std::atomic<bool>` 安全终止规划线程。
* **规划线程调度**:
    - 标准模式：当目标队列非空且处于自瞄模式时，以 **10ms** 周期运行；否则休眠 **200ms** 以降低 CPU 占用。
    - 调试模式：固定 **10ms** 周期运行，保证数据记录连续性。
* **时间戳同步**: 所有传感器数据（图像、陀螺仪）必须关联同一基准下的全局时间戳，精度要求 $\mu s$ 级。图像时间戳通过 `camera.read(img, t)` 获取，云台四元数通过 `gimbal.q(t - 6ms)` 进行时间补偿。

### 6.2 线程职责划分
| 线程 | 职责 | 优先级 | 周期 |
| :--- | :--- | :--- | :--- |
| **规划线程** | MPC 求解、指令发送 | 最高 | 10ms（有目标） / 200ms（空闲） |
| **串口读线程**（Gimbal 内部） | 接收下位机数据 | 高 | 1ms（硬件中断驱动） |
| **主线程** | 图像采集、检测、跟踪 | 中 | ~10ms（跟随相机帧率） |
| **串口写线程**（Gimbal 内部） | 发送指令到云台 | 中 | 1ms（定时发送） |

### 6.3 关键延迟节点
1. **图像传输延迟**：相机 → 内存，约 2-5ms。
2. **处理延迟**：YOLO 检测 + 跟踪 + 坐标变换，约 3-8ms。
3. **队列延迟**：目标队列传递，<1ms。
4. **规划延迟**：MPC 求解，约 1-2ms。
5. **指令传输延迟**：串口发送 + 下位机处理，约 1-2ms。

**总延迟**：目标 <33ms（3 帧内），满足实时控制要求。

### 6.4 性能与实时性约束
* **计算频率**:
    * 视觉处理（检测+跟踪）：$\ge 100\text{Hz}$（与相机帧率匹配）。
    * 云台控制（MPC）：**100Hz**（规划线程固定 10ms 周期）。
    * 底层通信（CAN 总线）：$1\text{ms}$ 周期。
* **端到端延迟**: 从图像采集到云台指令发送必须控制在 **33ms** 以内（即 3 帧内）。
* **调试开销**: JSON 数据记录与重投影可视化会引入额外 CPU 负载，建议仅在调试模式下启用。
* **CPU 亲和性**: 建议将视觉线程绑定至特定核心（i7-1260P 的性能核）。

### 6.5 错误处理与降级
- **目标丢失**：`target_queue.push(std::nullopt)`，规划线程收到空目标后发送停止指令。
- **模式切换**：通过 `gimbal.mode()` 实时切换处理流水线，确保控制逻辑与硬件状态同步。
- **串口断开**：`Gimbal` 内部自动重连，同时视觉模块继续运行（支持无硬件测试）。

---
## 第七部分：调试与可视化

### 7.1 调试机制
* **数据记录**:
  - **标准模式**: 使用 `tools::Recorder` 记录图像、云台四元数与时间戳，供离线回放测试。
  - **调试模式**: 使用 `tools::Plotter` 将以下数据实时输出为 JSON 格式：
    * 云台状态：`gimbal_yaw`、`gimbal_yaw_vel`、`gimbal_pitch`、`gimbal_pitch_vel`。
    * 目标状态：`target_z`、`target_vz`、`w`（角速度）。
    * 规划输出：`plan_yaw`、`plan_yaw_vel`、`plan_yaw_acc`、`plan_pitch`、`plan_pitch_vel`、`plan_pitch_acc`。
    * 开火状态：`fire`、`fired`（实际击发检测）。
    * 原始观测：`armor_num`、`armor_x`、`armor_y`、`armor_z`、`armor_yaw`、`armor_center_x`、`armor_center_y`。

### 7.2 调试字段详解（坐标系、物理意义与单位）
以下为 `src/auto_aim_debug_mpc.cpp` 中 `tools::Plotter` 输出的 JSON 字段详细说明。

| 类别 | 字段 | 坐标系 | 物理意义 | 单位 | 代码索引/备注 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **云台状态** | `gimbal_yaw` | 云台坐标系 | 云台当前偏航角（绕垂直轴旋转） | **rad** | `GimbalState::yaw` |
| | `gimbal_yaw_vel` | 云台坐标系 | 云台当前偏航角速度 | **rad/s** | `GimbalState::yaw_vel` |
| | `gimbal_pitch` | 云台坐标系 | 云台当前俯仰角（绕水平轴旋转） | **rad** | `GimbalState::pitch` |
| | `gimbal_pitch_vel` | 云台坐标系 | 云台当前俯仰角速度 | **rad/s** | `GimbalState::pitch_vel` |
| **目标状态** | `target_z` | 世界坐标系 | 目标**旋转中心**的 Z 坐标（高度） | **m** | `target->ekf_x()[4]` |
| | `target_vz` | 世界坐标系 | 目标**旋转中心**的 Z 方向速度 | **m/s** | `target->ekf_x()[5]` |
| | `w` | 世界坐标系 | 目标**旋转角速度**（绕旋转中心） | **rad/s** | `target->ekf_x()[7]` |
| | `target_yaw` | 云台坐标系 | MPC 计算的**目标偏航角**（瞄准点） | **rad** | `Plan::target_yaw` |
| | `target_pitch` | 云台坐标系 | MPC 计算的**目标俯仰角**（瞄准点） | **rad** | `Plan::target_pitch` |
| **规划输出** | `plan_yaw` | 云台坐标系 | MPC 输出的**当前控制偏航角** | **rad** | `Plan::yaw` |
| | `plan_yaw_vel` | 云台坐标系 | MPC 输出的**当前偏航角速度** | **rad/s** | `Plan::yaw_vel` |
| | `plan_yaw_acc` | 云台坐标系 | MPC 输出的**当前偏航角加速度** | **rad/s²** | `Plan::yaw_acc` |
| | `plan_pitch` | 云台坐标系 | MPC 输出的**当前控制俯仰角** | **rad** | `Plan::pitch` |
| | `plan_pitch_vel` | 云台坐标系 | MPC 输出的**当前俯仰角速度** | **rad/s** | `Plan::pitch_vel` |
| | `plan_pitch_acc` | 云台坐标系 | MPC 输出的**当前俯仰角加速度** | **rad/s²** | `Plan::pitch_acc` |
| **开火状态** | `fire` | – | **规划器开火请求** | – | `1`=请求开火，`0`=不开火 |
| | `fired` | – | **实际击发检测** | – | `1`=子弹计数增加，`0`=未击发 |
| **原始观测** | `armor_num` | – | 当前帧检测到的**装甲板数量** | 个 | – |
| | `armor_x` | 世界坐标系 | 第一个装甲板的 **X 坐标** | **m** | `armor.xyz_in_world[0]` |
| | `armor_y` | 世界坐标系 | 第一个装甲板的 **Y 坐标** | **m** | `armor.xyz_in_world[1]` |
| | `armor_z` | 世界坐标系 | 第一个装甲板的 **Z 坐标**（高度） | **m** | `armor.xyz_in_world[2]` |
| | `armor_yaw` | 世界坐标系 | 第一个装甲板的**优化后偏航角**（朝向） | **°** | `armor.ypr_in_world[0] * 57.3` |
| | `armor_yaw_raw` | 世界坐标系 | 第一个装甲板的**原始偏航角**（PnP 初值） | **°** | `armor.yaw_raw * 57.3` |
| | `armor_center_x` | 图像归一化坐标 | 装甲板中心点的 **X 方向比例** | 无量纲 | `center.x / 图像宽度`，范围 `[0,1]` |
| | `armor_center_y` | 图像归一化坐标 | 装甲板中心点的 **Y 方向比例** | 无量纲 | `center.y / 图像高度`，范围 `[0,1]` |

**关键说明**：
1. **坐标系定义**：
   - **世界坐标系**：原点在机器人当前位置，XY 平面为地面，Z 轴垂直向上。
   - **云台坐标系**：原点在云台旋转中心，X 轴向前（相机光轴），Y 轴向左，Z 轴向上。
   - **图像归一化坐标**：像素坐标除以图像宽高，用于视场角转换（`delta_angle = fov/2 - center_norm.x * fov`）。
2. **目标状态与观测的区别**：
   - `target_z`/`target_vz` 是**旋转中心**的 EKF 估计状态。
   - `armor_z` 是**装甲板本身**的原始观测值（未经滤波）。
3. **开火检测**：`fired` 通过比较 `GimbalState::bullet_count` 的增量判断实际击发。
4. **调试用途**：比较 `target_yaw`（期望）与 `plan_yaw`（实际）可评估 MPC 跟踪精度；`w`（角速度）影响开火决策阈值。

### 7.3 重投影可视化
- 使用 `planner.debug_xyza` 获取 MPC 选择的瞄准点（世界坐标系）。
- 通过 `solver.reproject_armor()` 将 3D 点投影回图像，绘制为红色点。
- 实时显示缩小的图像窗口，按 `q` 退出。

### 7.4 离线回放支持
```cpp
recorder.record(img, q, t);  // 记录图像、四元数、时间戳到文件
```
- **格式**：自定义二进制格式，可通过 `tools/recorder.hpp` 回放，用于无硬件测试。

---
## 第八部分：特殊模式

### 8.1 打符（Buff）模式
* **检测与求解**: 使用 `auto_buff::Buff_Detector` 检测能量机关，`auto_buff::Solver` 求解目标位姿。
* **目标分类**: 区分 `SmallTarget`（小符）与 `BigTarget`（大符），分别调用 `buff_small_target.get_target()` 与 `buff_big_target.get_target()`。
* **MPC 规划**: 通过 `buff_aimer.mpc_aim()` 生成云台轨迹，其余控制接口与自瞄模式一致。

#### 8.1.1 模式检测
```cpp
mode = gimbal.mode();  // 读取下位机发送的模式字节
if (mode.load() == io::GimbalMode::SMALL_BUFF || mode.load() == io::GimbalMode::BIG_BUFF) {
    // 打符处理逻辑
}
```

#### 8.1.2 能量机关检测与求解
```cpp
auto power_runes = buff_detector.detect(img);  // 检测能量机关叶片
buff_solver.solve(power_runes);  // 求解叶片在世界坐标系下的位姿
```

#### 8.1.3 目标分类与预测
- **小符**：`buff_small_target.get_target(power_runes, t)` 预测激活叶片。
- **大符**：`buff_big_target.get_target(power_runes, t)` 预测运动叶片。
- **EKF 模型**：与自瞄类似，但状态向量针对旋转运动优化。

#### 8.1.4 MPC 规划
```cpp
auto buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
```
- **接口统一**：返回 `auto_aim::Plan` 结构体，控制流与自瞄完全一致。
- **专用参数**：打符的 MPC 权重、约束在 `configs/standard3.yaml` 的 `buff_aimer` 配置节中独立设置。

### 8.2 调试模式 (`src/auto_aim_debug_mpc.cpp`)
- **固定周期规划**：始终以 **10ms** 周期执行规划线程，保证数据记录连续性。
- **完整数据记录**：通过 `tools::Plotter` 输出 JSON 格式的调试数据（见 §7.2）。
- **重投影可视化**：实时显示瞄准点重投影，用于视觉-控制对齐验证。
- **应用场景**：算法调试、参数调优、性能分析、坐标系验证。

---
## 第九部分：完整实现参考

### 9.1 标准 MPC 自瞄完整数据链路 (`src/standard_mpc.cpp`)
本节点实现机器人自瞄与打符的完整闭环控制，数据从相机与下位机流入，经坐标变换、检测、跟踪、滤波、规划，最终生成云台指令发送给下位机。以下按**硬件输入 → 坐标变换 → 视觉处理 → 状态估计 → 轨迹规划 → 控制输出**的顺序概述完整链路，各环节详细说明请参见前述章节。

#### 9.1.1 硬件数据输入
- **相机图像采集**：`camera.read(img, t)` 阻塞读取图像与硬时间戳（§1.1、§6.1）。
- **云台状态反馈**：`gimbal.q(t - 6ms)` 获取时间补偿后的四元数，`gimbal.state()` 获取角度、角速度、弹速、弹计数（§3.3、§6.1）。

#### 9.1.2 坐标变换链
1. **手眼标定矩阵**：`R_camera2gimbal_`、`t_camera2gimbal_`、`R_gimbal2imubody_`（§2.2）。
2. **世界坐标系建立**：`solver.set_R_gimbal2world(q)` 根据云台四元数实时更新世界坐标系旋转矩阵（§2.4）。
3. **像素坐标 → 世界坐标**：`solver.solve(armor)` 完成 PnP 解算与坐标系转换（§2.3、§4.1）。

#### 9.1.3 视觉处理流水线
1. **YOLO 检测**：`yolo.detect(img)` 输出 `Armor` 列表（§4.2）。
2. **多目标跟踪**：`tracker.track(armors, t)` 关联装甲板与目标，管理目标状态机（§4.3）。
3. **扩展卡尔曼滤波**：`Target::update(armor)` 更新目标旋转中心状态估计（§4.4）。

#### 9.1.4 轨迹规划（MPC）
1. **规划线程调度**：目标队列非空且处于自瞄模式时以 **10ms** 周期运行（§5.2、§6.1）。
2. **弹道补偿与瞄准点计算**：`Planner::aim()` 计算世界坐标系下的目标角度（§2.5、§2.6、§5.3）。
3. **TinyMPC 轨迹生成**：分别求解 yaw/pitch 轴轨迹，生成 1 秒平滑轨迹（§5.4）。
4. **开火决策**：根据目标角速度与跟踪误差判断是否开火（§5.5）。

#### 9.1.5 控制指令输出
- **指令封装**：`gimbal.send(control, fire, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc)`（§5.6）。
- **协议发送**：通过串口发送 `VisionToGimbal` 数据包，或通过 CAN 总线发送控制帧（§3.1）。

#### 9.1.6 打符模式处理
- **模式检测**：读取 `gimbal.mode()` 切换至打符逻辑（§8.1）。
- **能量机关检测**：`buff_detector.detect(img)` 与 `buff_solver.solve(power_runes)`（§8.1.2）。
- **专用 MPC 规划**：`buff_aimer.mpc_aim()` 返回 `Plan` 结构体（§8.1.4）。

#### 9.1.7 数据记录与调试
- **离线回放**：`recorder.record(img, q, t)` 支持无硬件测试（§7.4）。
- **实时调试**：`tools::Plotter` 输出 JSON 数据，`solver.reproject_armor()` 可视化瞄准点（§7.1、§7.3）。

### 9.2 关键配置文件 (`configs/standard3.yaml`)
- **相机参数**：`camera_matrix`、`distort_coeffs`、`R_camera2gimbal`、`t_camera2gimbal`。
- **控制参数**：`yaw_offset`、`pitch_offset`、`decision_speed`、`fire_thresh`。
- **MPC 参数**：`Q_yaw`、`R_yaw`、`max_yaw_acc`、`Q_pitch`、`R_pitch`、`max_pitch_acc`。
- **打符参数**：`buff_aimer` 配置节。
- **通信参数**：`com_port`、`quaternion_canid`、`bullet_speed_canid`、`send_canid`。

### 9.3 Dashboard 热参数接口
- **模型入口**：`tools::dashboard::DashboardParams` 生成 `params/schema` 与 `params/current` payload，并校验单个参数更新请求。
- **MQTT 发布接口**：完整 `params/schema`、`params/current` envelope 由 `MqttBridge::publish_params_schema_payload()` 与 `MqttBridge::publish_params_current_payload()` 发布。
- **可热改范围**：仅包含 `planner.yaw_offset_deg`、`planner.pitch_offset_deg`、`planner.fire_thresh`、`planner.decision_speed`、`planner.high_speed_delay_time`、`planner.low_speed_delay_time`、`buff.yaw_offset_deg`、`buff.pitch_offset_deg`、`buff.fire_gap_time`、`buff.predict_time`。
- **线程安全接口**：`auto_aim::Planner::get_hot_params()` / `apply_hot_param()` 与 `auto_buff::Aimer::get_hot_params()` / `apply_hot_param()` 使用 mutex + RAII；offset 对外为 degree，内部仍存 rad。
- **边界**：Dashboard 热调参不读取 MQTT，不依赖 Paho，不热改 TinyMPC `Q/R/max_acc`，不重建 solver。

### 9.4 构建与运行命令
```bash
# 构建发布版本
cmake -B build -DCMAKE_BUILD_TYPE=Release && make -C build/ -j$(nproc)

# 运行标准 MPC 自瞄+打符
make -C build/ standard_mpc && ./build/standard_mpc configs/standard3.yaml

# 运行调试 MPC（JSON 数据记录 + 重投影可视化）
make -C build/ auto_aim_debug_mpc && ./build/auto_aim_debug_mpc configs/standard3.yaml
```

---

## 文档使用指南

1. **新开发者入门**：阅读 §1 系统概述，理解数据流与分层架构；重点掌握 §2 坐标系与变换，避免常见错误。
2. **算法修改**：修改坐标相关代码前必读 §2；修改视觉处理流程参考 §4；修改轨迹规划参考 §5。
3. **通信协议开发**：与下位机对接时参考 §3，明确坐标系约定与控制指令语义。
4. **性能调优**：参考 §6 多线程与实时性约束，确保端到端延迟 <33ms。
5. **问题调试**：使用 §7 调试机制，通过 JSON 数据记录与重投影可视化定位问题。
6. **特殊模式开发**：打符模式参考 §8，调试模式参考 `src/auto_aim_debug_mpc.cpp`。

> **版本说明**：本文档基于原 `KNOWLEDGE.md` 重组优化，内容完整性保持不变，结构调整为九大逻辑部分，提升可读性与查找效率。原章节编号已更新，内部引用相应调整。

---
