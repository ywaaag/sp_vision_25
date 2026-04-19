# 故障排除与系统维护

## 1. Gimbal 串口链路排查

### 1.1 构造阶段卡住
现象：
- 程序启动后停在 `io::Gimbal` 构造附近。
- 日志只看到串口打开，没有看到 `[Gimbal] First q received.`。

原因：
- `Gimbal` 构造函数会先启动读线程，再阻塞等待第一帧四元数进队列。

排查：
```bash
ls -l /dev/gimbal
```
```bash
grep -n "com_port" configs/standard3.yaml
```
- 确认下位机确实在发 `0x5A 0x01` 上行包。
- 确认串口设备名与 YAML 中的 `com_port` 一致。

### 1.2 串口反复重连
现象：
- 日志反复出现 `Serial read abnormal for too long, attempting to reconnect...`

源码事实：
- `read_thread()` 连续读异常超过 2 秒触发 `reconnect()`。
- 最多重试 10 次。

排查：
- 检查线缆、供电和设备枚举是否稳定。
- 检查下位机是否持续发送完整帧，而不是只发包头。
- 检查串口权限和设备占用。

## 2. 协议格式错误

### 2.1 `Invalid package head`
现象：
- 日志报 `[Gimbal] Invalid package head`。

含义：
- 第一字节不是 `0x5A`，或者第二字节不是当前代码支持的 `0x01` / `0x02` / `0x03`。

排查：
- 上行视觉包必须是 `0x5A 0x01`。
- 裁判系统包 1 必须是 `0x5A 0x02`。
- 裁判系统包 2 必须是 `0x5A 0x03`。

### 2.2 `Invalid tail`
现象：
- 日志报 `Invalid tail`。

关键事实：
- 上行包要求 `tail == 0x55`。
- 下行包也是 `0x55`，但其位置与上行不同。

最容易错的地方：
- 上行 `GimbalToVision` 是 `... tail, check_sum`。
- 下行 `VisionToGimbal` / `NavToGimbal` 是 `... check_sum, tail`。

### 2.3 `Invalid check sum`
现象：
- 日志报 `Invalid check sum`。

关键事实：
- 上行校验函数 `verify_check_sum16()` 把最后两个字节解释为校验和。
- 下行写校验时，`append_check_sum()` 把和写到倒数第 3、倒数第 2 个字节，最后一个字节留给 `tail`。

排查重点：
- 不要把上行和下行的 `tail/check_sum` 排列写反。
- 不要改掉 `packed` 布局。
- 检查下位机是否按“逐字节求和 -> 16 位截断”的方式生成校验。

## 3. 字段语义排查

### 3.1 模式切换不对
现象：
- 视觉端模式一直不对，或者打符/自瞄切换错误。

上行 `mode` 映射：
- `0 -> IDLE`
- `1 -> AUTO_AIM`
- `2 -> SMALL_BUFF`
- `3 -> BIG_BUFF`

排查：
- 先看下位机实际发送值。
- 再看 `gimbal.mode()` 在主线程里的读取结果。

### 3.2 弹速异常
现象：
- 自瞄角度明显不对，或日志里弹速一直是默认值。

源码事实：
- `Planner` 把 `bullet_speed < 10` 或 `> 25` 视为异常并回退到 `22.5`。
- `Aimer` 把 `bullet_speed < 14` 回退到 `23`。

排查：
- 检查上行 `bullet_speed` 是否正常刷新。
- 不要只盯着下发角度，先确认 `gimbal.state().bullet_speed`。

### 3.3 `fired` 一直为 0
现象：
- `auto_aim_debug_mpc` 里 `fire` 有变化，但 `fired` 一直为 0。

源码事实：
- `fired` 不是下位机直接发的布尔值。
- 它是通过 `bullet_count` 是否递增推断出来的。

排查：
- 检查上行 `bullet_count` 是否递增。
- 检查是否只是 `plan.fire == true`，但下位机未实际发弹。

### 3.4 `yaw_diff` / `DWT_stamp` 看起来没生效
这是正常现象。

源码事实：
- 当前视觉侧没有消费 `yaw_diff`。
- 当前视觉侧也没有用 `DWT_stamp` 做时间同步。
- 当前对齐时间来自主机侧接收时刻和 `gimbal.q(t - 6ms)`。

## 4. 图像与姿态对齐问题

### 4.1 重投影明显偏
优先检查：
1. `gimbal.q(t - 6ms)` 里的 `6ms` 是否仍然适用。
2. `R_camera2gimbal`
3. `t_camera2gimbal`
4. `R_gimbal2imubody`

相关命令：
```bash
grep -n "R_camera2gimbal" configs/standard3.yaml
grep -n "t_camera2gimbal" configs/standard3.yaml
grep -n "R_gimbal2imubody" configs/standard3.yaml
```

### 4.2 角度方向看起来反了
关键事实：
- `tools::Trajectory::pitch` 定义为抬头为正。
- 但下发给 `gimbal.send(...)` 的控制 `pitch` 是：
```cpp
-bullet_traj.pitch - pitch_offset
```

结论：
- 观测/弹道里的 pitch 符号和控制接口里的 pitch 符号不是一回事。
- 排查时不要把这两个量直接当成同义字段。

## 5. 运行命令

### 5.1 自瞄串口调试
```bash
make -C build auto_aim_debug_mpc
./build/auto_aim_debug_mpc configs/standard3.yaml
```

### 5.2 打符串口调试
```bash
make -C build auto_buff_debug_mpc
./build/auto_buff_debug_mpc configs/standard3.yaml
```

### 5.3 标准串口主程序
```bash
make -C build standard_mpc
./build/standard_mpc configs/standard3.yaml
```

说明：
- 当前仓库里没有 `standard_debug` 这个目标，不要再用这个旧命令排查。

## 6. 容器、图形和相机联调排查

### 6.1 `standard_mpc` 一启动就报缺少 ROS2 动态库
现象：
- 启动 `./build/standard_mpc configs/standard3.yaml` 后，直接报类似：
```text
error while loading shared libraries: librcl_interfaces__rosidl_typesupport_cpp.so: cannot open shared object file
```

原因：
- 这通常不是二进制没编出来，而是运行该命令时没有加载完整 ROS2 运行环境。
- 当前容器里 `standard_mpc` 依赖两层环境：
  - `/opt/ros/humble/setup.bash`
  - `/root/Combat_Sentry2026/combat_nav_ws/install/setup.bash`

排查：
```bash
docker exec -it Combat_Sentry2026 bash
cd /root/Combat_Sentry2026/sp_vision_25
./build/standard_mpc configs/standard3.yaml
```
- 若不是通过交互式 shell 进入容器，手动执行：
```bash
source /opt/ros/humble/setup.bash
source /root/Combat_Sentry2026/combat_nav_ws/install/setup.bash
```

### 6.2 `standard_mpc` 看起来“没有运行”
现象：
- 没有 `imshow` 窗口。
- 终端输出很少，似乎卡住。

源码事实：
- `src/standard_mpc.cpp` 当前没有 `cv::imshow(...)`。
- 它是无界面主程序，主要通过日志、ROS2 和录包工作。
- 运行后会在 `logs/` 和 `records/` 下生成新文件。

排查：
```bash
ls -lt logs | head
ls -lt records | head
```
- 若能看到新的 `.log`、`.avi`、`.txt`，通常说明主程序已经在工作。

### 6.3 `camera_test` 默认测的不是海康
现象：
- 直接运行 `./build/camera_test` 后报找不到相机，或者行为和主程序不一致。

原因：
- `tests/camera_test.cpp` 的默认配置是 `configs/camera.yaml`。
- 当前仓库里的 `configs/camera.yaml` 默认走的是迈德威视，不是 `configs/standard3.yaml` 里的海康配置。

排查：
```bash
./build/camera_test --config-path=configs/standard3.yaml
```
- 需要显示图像时，再追加 `--display`。

### 6.4 海康相机报 `MV_CC_OpenDevice failed: 0x80000203`
现象：
- 日志出现：
```text
MV_CC_OpenDevice failed: 0x80000203
```

关键事实：
- `0x80000203` 对应 `MV_E_ACCESS_DENIED`。
- 当前容器内实测，海康 SDK 日志会进一步显示 `usb_claim_interface failed`。

最常见原因：
- 另一个进程已经占住相机，例如同时运行了 `standard_mpc`、`auto_aim_debug_mpc`、`camera_test`、MVS 或其它相机程序。

排查：
- 不要并发运行多个会打开海康相机的程序。
- 若需要最小化验证相机链路，先停掉主程序，再单独运行：
```bash
./build/camera_test --config-path=configs/standard3.yaml
```
- SDK 自身日志优先看：
```bash
tail -n 100 MvSdkLog/CamCtrl_00.log
tail -n 100 MvSdkLog/MvUsb3vTL_00.log
```

### 6.5 海康相机报 `MV_CC_EnumDevices failed: 0x80000006`
现象：
- 日志连续出现：
```text
MV_CC_EnumDevices failed: 0x80000006
```

当前结论：
- 这类报错在本项目里出现过，但不能单独据此得出“容器没挂相机”或“MVS 没装好”。
- 2026-04-15 / 2026-04-16 的后续实测里，同一容器又成功跑通了 `camera_test --config-path=configs/standard3.yaml`、`standard_mpc configs/standard3.yaml` 和 `auto_aim_debug_mpc configs/standard3.yaml`。
- 因此更稳妥的判断顺序是：
  1. 先单独、串行验证最小程序。
  2. 再排查是否有旧进程未退出、USB 状态抖动或前一轮异常未清干净。
  3. 最后才考虑 SDK 安装和设备挂载问题。

建议：
- 先停掉所有可能占用海康的进程，再单独运行 `camera_test`。
- 若问题复现，再同时看 `MvSdkLog/` 和 `dmesg` / `lsusb` / 容器内设备状态。

### 6.6 `auto_aim_debug_mpc` 报 X11 / GTK 初始化失败
现象：
- `standard_mpc` 能运行，但 `auto_aim_debug_mpc`、`auto_buff_debug_mpc` 等带显示窗口的调试程序在容器里直接退出。
- 常见日志：
```text
Authorization required, but no authorization protocol specified
*******XOpenDisplay Fail *******
terminate called after throwing an instance of 'cv::Exception'
what(): OpenCV ... Can't initialize GTK backend
```

原因：
- 这通常不是相机、OpenCV 或模型本身坏了。
- 当前宿主机已经有 `DISPLAY`，但容器内的 `root` 没有当前桌面会话的 X11 授权。
- 因为 `auto_aim_debug_mpc` 会调用 `cv::imshow(...)`，GTK 初始化阶段会因为授权失败而抛异常。

关键判断：
- `Authorization required, but no authorization protocol specified` 更接近“显示服务可达，但没有权限”，不是简单的环境变量没传。
- 对当前这台机器和当前这只容器，优先级最高的修复不是重建容器，而是先在宿主机执行 `xhost +SI:localuser:root`。

推荐做法：
在宿主机执行：
```bash
xhost +SI:localuser:root
docker start Combat_Sentry2026
docker exec -it -e DISPLAY="$DISPLAY" Combat_Sentry2026 bash
```

进入容器后执行：
```bash
cd /root/Combat_Sentry2026/sp_vision_25
./build/auto_aim_debug_mpc configs/standard3.yaml
```

可选后备方案：
- 若只靠 `xhost +SI:localuser:root` 仍然失败，再退回到把当前桌面会话 cookie 复制进容器：
```bash
docker cp "$XAUTHORITY" Combat_Sentry2026:/root/.Xauthority
docker exec -it \
  -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY=/root/.Xauthority \
  Combat_Sentry2026 bash
```
- `XAUTHORITY` 是宿主机当前图形会话的 X11 授权文件路径。在本机当前环境里，它通常位于 `/run/user/1000/.mutter-Xwaylandauth.*` 一类动态文件；不要把文件名写死，始终直接使用宿主机当前 shell 的 `$XAUTHORITY`。

安全提醒：
- 不建议使用 `xhost +`，它会直接关闭访问控制，权限过大。
- 联调结束后建议在宿主机执行：
```bash
xhost -SI:localuser:root
```

### 6.7 `auto_aim_debug_mpc` 启动后仍出现 GTK 警告
现象：
- 程序已经能跑，但终端还会出现：
```text
dbind-WARNING **: Couldn't connect to accessibility bus
Gtk-Message: Failed to load module "canberra-gtk-module"
```

当前结论：
- 2026-04-16 实测，这两类日志不会阻止窗口显示，也不会阻止海康采图线程和主循环继续工作。
- 它们当前归类为非阻塞警告，不要误判为导致 `imshow` 失败的主因。

## 7. 2026-04-15 / 2026-04-16 容器联调实测纪要

### 7.1 本轮联调的进入方式
推荐进入方式：
```bash
docker start Combat_Sentry2026
docker exec -it Combat_Sentry2026 bash
cd /root/Combat_Sentry2026/sp_vision_25
```

关键事实：
- 当前容器的 `~/.bashrc` 会自动加载：
```bash
source /opt/ros/humble/setup.bash
source /root/Combat_Sentry2026/combat_nav_ws/install/setup.bash
```
- 这件事非常关键：`standard_mpc` 的编译和运行都依赖完整 ROS2 运行环境。若改用 `docker exec ... bash -lc` 这类非交互方式，可能出现“能编一部分、但运行时缺动态库”的假问题。

### 7.2 当前容器已确认的事实
基于 `docker inspect Combat_Sentry2026` 和容器内实测，当前容器已经具备：
- `Privileged=true`
- `NetworkMode=host`
- 绑定 `/dev:/dev`
- 绑定 `/tmp/.X11-unix:/tmp/.X11-unix`
- 默认环境变量含 `DISPLAY=:0`
- 工作区挂载：
  - `/home/zero/桌面/combat_sentry/sp_vision_25 -> /root/Combat_Sentry2026/sp_vision_25`
  - `/home/zero/桌面/combat_sentry/CombatNav -> /root/Combat_Sentry2026/combat_nav_ws/src`

由此得到的排障结论：
- 当前 `auto_aim_debug_mpc` 的图形问题，不是“容器没挂 X11 socket”，而是“宿主机没授权容器内的 `root` 打开显示”。
- 当前问题下不需要为了图形转发单独创建 `combat_sentry_v2` / `Combat_Sentry2026_v2`；只有当后续需要隔离依赖改动、固化新环境或追加新的宿主机挂载策略时，新容器才有意义。

### 7.3 MVS / 海康 SDK 环境的实测事实
- 容器内 `lsusb` 能看到海康相机：`2bdf:0001 U3V MV-CS016-10UC`。
- 容器内存在完整 MVS 导出环境：
  - `MVCAM_SDK_PATH=/opt/MVS`
  - `MVCAM_COMMON_RUNENV=/opt/MVS/lib`
  - `LD_LIBRARY_PATH` 包含 `/opt/MVS/lib/64`
- `auto_aim_debug_mpc`、`camera_test`、`standard_mpc` 实际使用的是 `/opt/MVS/lib/64/libMvCameraControl.so`，不是仓库里自带的备份 so。
- 容器是 `privileged=true`，并且绑定了整个 `/dev`，因此“USB 根本没挂进容器”不是当前主问题。

### 7.4 `camera_test` 的正确打开方式和实测结果
正确命令：
```bash
./build/camera_test --config-path=configs/standard3.yaml
```

原因：
- `tests/camera_test.cpp` 默认配置是 `configs/camera.yaml`。
- 当前仓库的 `configs/camera.yaml` 默认走迈德威视，不是当前海康主链路。

2026-04-15 实测结果：
- 海康守护线程正常启动。
- 采集线程正常启动。
- 单独运行时稳定在约 `23.5 ~ 25 fps`。

结论：
- “容器内完全看不到海康相机”这个判断不成立。
- 直接运行 `./build/camera_test` 的结果不能直接推导当前海康主链路是否损坏。

### 7.5 `standard_mpc` 的正确打开方式和实测结果
正确命令：
```bash
./build/standard_mpc configs/standard3.yaml
```

2026-04-15 / 2026-04-16 关键日志：
```text
[Gimbal] read_thread started.
[Gimbal] First q received.
HikRobot's daemon thread started.
[referee_publisher]: referee_publisher initialized.
[nav_cmd_subscriber]: nav_cmd_subscriber initialized.
[ROS2]: ROS2 initialized.
Switch to SMALL_BUFF
HikRobot's capture thread started.
```

结论：
- 当容器环境正确、且没有其它进程争用相机时，`standard_mpc` 可以：
  - 完成 ROS2 初始化。
  - 读取串口姿态。
  - 启动海康采图线程。
  - 进入主循环。
- 它当前没有图像显示窗口，这不是 bug，而是程序设计。

### 7.6 `auto_aim_debug_mpc` 的失败与修复过程
#### 7.6.1 初始失败现象
在宿主机只执行普通 `docker exec` 进入容器后，运行：
```bash
./build/auto_aim_debug_mpc configs/standard3.yaml
```
曾出现：
```text
Authorization required, but no authorization protocol specified
*******XOpenDisplay Fail *******
HikRobot's capture thread started.
Authorization required, but no authorization protocol specified
terminate called after throwing an instance of 'cv::Exception'
what(): OpenCV ... Can't initialize GTK backend
```

解释：
- 海康线程其实已经起来了。
- 真正让程序中止的是 GTK / X11 初始化失败，而不是海康相机没工作。

#### 7.6.2 修复命令
宿主机执行：
```bash
xhost +SI:localuser:root
docker start Combat_Sentry2026
docker exec -it -e DISPLAY="$DISPLAY" Combat_Sentry2026 bash
```

容器内执行：
```bash
cd /root/Combat_Sentry2026/sp_vision_25
./build/auto_aim_debug_mpc configs/standard3.yaml
```

#### 7.6.3 修复后的实测结果
程序随后正常启动并持续运行，终端可见：
```text
[Gimbal] read_thread started.
[Gimbal] First q received.
HikRobot's daemon thread started.
HikRobot's capture thread started.
[Target] r=0.510, l=0.000
[Tracker] Target diverged!
```

结论：
- 容器图形层已经不是当前主阻塞。
- 当前问题已经推进到了算法联调层，后续应把注意力转到检测、目标选择、Tracker 和配置参数。

### 7.7 并发占用相机时会出现什么
本轮联调里出现过两类海康错误：
1. `MV_CC_EnumDevices failed: 0x80000006`
2. `MV_CC_OpenDevice failed: 0x80000203`

其中第二类已有更明确证据：
- `0x80000203 == MV_E_ACCESS_DENIED`
- `MvSdkLog/MvUsb3vTL_00.log` 明确出现：
```text
[OpenDevice][usb_claim_interface] failed
```

在 2026-04-15 的实测里，这个错误发生在并发运行 `standard_mpc` 和 `camera_test` 时。

结论：
- 当前最确定的触发条件是“多个进程同时打开同一台海康相机”。
- 常见冲突者包括：
  - `standard_mpc`
  - `auto_aim_debug_mpc`
  - `camera_test`
  - MVS / MvToolkit
  - 其它任何调用海康 SDK 的程序

因此如果再次看到 `0x80000203`，第一反应不是重装 SDK，而是先排除并发占用。

### 7.8 这轮联调应优先保留的日志文件
- 主程序日志：
  - `logs/<timestamp>.log`
- 录包：
  - `records/<timestamp>.txt`
  - `records/<timestamp>.avi`
- 海康 SDK 日志：
  - `MvSdkLog/CamCtrl_00.log`
  - `MvSdkLog/MvUsb3vTL_00.log`

建议：
- 若明天继续联调，优先先看这些文件，再决定是否重复跑程序。

## 8. 下一轮联调的推荐顺序

### 8.1 先确认没有遗留占用进程
```bash
ps -ef | grep -E "standard_mpc|auto_aim_debug_mpc|camera_test|MVS|MvToolkit" | grep -v grep
```

### 8.2 先用最小程序验相机
```bash
./build/camera_test --config-path=configs/standard3.yaml
```
- 期望结果：海康守护线程和采集线程启动，fps 约 24。

### 8.3 再单独运行 `standard_mpc`
```bash
./build/standard_mpc configs/standard3.yaml
```
- 重点看：
  - 是否出现 `ROS2 initialized.`
  - 是否出现 `Switch to ...`
  - 是否出现 `HikRobot's capture thread started.`

### 8.4 再运行需要窗口的调试程序
宿主机：
```bash
xhost +SI:localuser:root
docker start Combat_Sentry2026
docker exec -it -e DISPLAY="$DISPLAY" Combat_Sentry2026 bash
```

容器内：
```bash
cd /root/Combat_Sentry2026/sp_vision_25
./build/auto_aim_debug_mpc configs/standard3.yaml
```

### 8.5 若仍觉得“程序没跑”
直接检查：
```bash
ls -lt logs | head
ls -lt records | head
tail -n 100 logs/最新日志文件
tail -n 100 MvSdkLog/CamCtrl_00.log
tail -n 100 MvSdkLog/MvUsb3vTL_00.log
```

## 9. 维护规则
- 协议字段一旦变化，先改 `io/gimbal/gimbal.hpp` / `io/gimbal/gimbal.cpp`，再同步 `.agent/KNOWLEDGE.md`。
- 若新增新的上行/下行包类型，必须把包头、包长、字段、校验方式一起记入 `.agent/KNOWLEDGE.md`。
- 若调过姿态补偿延迟，必须同步 `.agent/KNOWLEDGE.md` 与 `.agent/TODO.md`。
- 若新增新的容器排障、相机占用特征或 GUI/X11 修复方法，优先补充到本文档，而不是写回 `.agent/DEVELOPMENT.md`。
