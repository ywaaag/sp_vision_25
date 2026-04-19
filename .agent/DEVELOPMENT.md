---

# 核心开发规范与工作流 (DEVELOPMENT.md)

## 1. 文档职责
- 本文档只维护开发规范、构建方式、运行入口和文档同步约束。
- 架构、协议、坐标链和算法事实看 `.agent/KNOWLEDGE.md`。
- 故障现象、排查路径、实测日志和联调记录看 `.agent/TROUBLESHOOTING.md`。
- 当前状态、待办和阶段性结论看 `.agent/TODO.md`。

## 2. 当前源码事实
- 语言标准：C++17。
- 主要依赖：OpenCV、fmt、Eigen3、spdlog、yaml-cpp、nlohmann_json、OpenVINO。
- `tasks/auto_buff` 额外依赖 Ceres。
- 当前 `io/CMakeLists.txt` 仍然对 `ament_cmake`、`rclcpp`、`std_msgs`、`rosidl_typesupport_cpp`、`combat_rm_interfaces`、`example_interfaces` 使用 `find_package(... REQUIRED)`；不要把仓库当成“完全无 ROS2 依赖”的状态。
- 当前 `tools/yaml.hpp` 只做两件事：`LoadFile` 和缺键报错退出；它没有实现统一的范围校验。
- 当前 CMake 没有全局开启 `-Wall -Wextra -Werror`；不要误以为这些告警选项已经落地。

## 3. 代码约束
- 禁止手写长期持有的 `new` / `delete`，优先 `std::unique_ptr` / `std::shared_ptr`。
- 跨线程队列优先使用项目内的 `tools::ThreadSafeQueue`。
- 日志统一走 `tools::logger()`，不要混用 `std::cout`。
- 矩阵、坐标变换与四元数运算统一走 Eigen。
- 命名沿用现有风格：
  - 类名 `PascalCase`
  - 函数和变量 `snake_case`
  - 常量 `UPPER_SNAKE_CASE`
- 提交前格式化：
```bash
find . -name "*.hpp" -o -name "*.cpp" | xargs clang-format -i
```

## 4. 目录职责
| 目录 | 当前职责 | 核心事实 |
| :--- | :--- | :--- |
| `io/` | 硬件与通信 | `gimbal` 走串口，`cboard` 走 CAN，`camera` 按 YAML 选择海康或迈德威视。 |
| `tools/` | 通用工具 | 包含 EKF、弹道、日志、JSON 画图、录包、校验和、线程安全队列。 |
| `tasks/auto_aim/` | 自瞄算法 | YOLO 检测、PnP/坐标变换、Tracker/Target、Aimer、TinyMPC Planner。 |
| `tasks/auto_buff/` | 打符算法 | Buff 检测、位姿解算、EKF 目标、打符瞄准。 |
| `tasks/omniperception/` | 全向感知原型 | 当前仍是原型，不直接等同于串口主线可用方案。 |
| `src/` | 主程序入口 | `standard_mpc` / `auto_aim_debug_mpc` 使用 `io::Gimbal`；`standard` / `mt_standard` 使用 `io::CBoard`。 |
| `configs/` | YAML 参数 | `configs/standard3.yaml` 是当前常用主配置。 |

## 5. Docker 与运行规范
- 本项目默认在 Docker 容器内构建、运行和联调，不直接在宿主机执行二进制。
- 当前约定容器名：`Combat_Sentry2026`。
- 当前镜像：`combat_sentry_v1:latest`。
- 当前仓库在容器内的挂载根目录是：`/root/Combat_Sentry2026/sp_vision_25`。
- 容器 ID 会变化，不要把 ID 写死到脚本或文档里，优先使用容器名。
- 宿主机侧标准进入方式：
```bash
docker start Combat_Sentry2026
docker exec -it Combat_Sentry2026 bash
```
- 进入容器后，所有构建和运行命令都从仓库根目录执行：
```bash
cd /root/Combat_Sentry2026/sp_vision_25
```
- 推荐使用交互式 shell 进入容器：`docker exec -it Combat_Sentry2026 bash`。当前容器的 `~/.bashrc` 会自动 `source /opt/ros/humble/setup.bash`，并在存在时继续 `source /root/Combat_Sentry2026/combat_nav_ws/install/setup.bash`；若改用非交互方式执行命令，需手动补齐这两步环境初始化。
- 需要图形窗口的调试程序，宿主机侧标准做法是先授权当前桌面会话给容器内 `root`：
```bash
xhost +SI:localuser:root
docker start Combat_Sentry2026
docker exec -it -e DISPLAY="$DISPLAY" Combat_Sentry2026 bash
```
- 图形联调结束后，建议在宿主机收回这次额外授权：
```bash
xhost -SI:localuser:root
```
- 若图形程序仍然报 X11 / GTK 错误，不要继续在本文档里查细节，直接看 `.agent/TROUBLESHOOTING.md`。
- `auto_aim_debug_mpc` 的默认 `config-path` 是相对路径 `configs/standard3.yaml`；`standard_mpc` 也按当前仓库习惯从根目录带配置参数启动。因此当前联调时必须先 `cd /root/Combat_Sentry2026/sp_vision_25`，再执行 `./build/...`。

## 6. 常用构建与运行命令
```bash
# 先进入容器内仓库根目录
cd /root/Combat_Sentry2026/sp_vision_25

# 配置
cmake -B build -DCMAKE_BUILD_TYPE=Release

# 全量编译
make -C build -j$(nproc)

# 串口自瞄调试（带窗口）
make -C build auto_aim_debug_mpc
./build/auto_aim_debug_mpc configs/standard3.yaml

# 海康相机最小化验证
make -C build camera_test
./build/camera_test --config-path=configs/standard3.yaml

# 串口打符调试（带窗口）
make -C build auto_buff_debug_mpc
./build/auto_buff_debug_mpc configs/standard3.yaml

# 标准串口主程序（无 GUI）
make -C build standard_mpc
./build/standard_mpc configs/standard3.yaml

# CAN 单线程自瞄
make -C build standard
./build/standard configs/standard3.yaml

# CAN 多线程自瞄 + 打符
make -C build mt_standard
./build/mt_standard configs/standard3.yaml

# 离线自瞄测试
make -C build auto_aim_test
./build/auto_aim_test assets/demo/demo -c configs/demo.yaml
```
- `camera_test` 默认配置是 `configs/camera.yaml`，当前仓库该文件默认走迈德威视；需要验证海康链路时，不要省略 `--config-path=configs/standard3.yaml`。
- `standard_mpc` 是无 GUI 主程序，正常运行时不弹 `imshow` 窗口；优先通过 `logs/`、`records/` 和 `MvSdkLog/` 判断运行状态。
- `auto_aim_debug_mpc`、`auto_buff_debug_mpc`、`mt_auto_aim_debug`、`uav_debug` 等调试程序会走 `cv::imshow(...)`；它们默认要求宿主机先完成 X11 授权。

## 7. 配置与文档同步约束
- 新增协议字段、控制参数或联调开关时，优先同步实际使用的配置文件，再决定是否回填到其它示例 YAML。
- 变更 gimbal 协议、延迟补偿或串口设备名后，必须同步 `.agent/KNOWLEDGE.md`。
- 变更运行流程、容器进入方式、构建方式或标准调试命令后，必须同步本文档。
- 新增稳定的报错特征、故障现象或修复路径后，必须同步 `.agent/TROUBLESHOOTING.md`。

---
