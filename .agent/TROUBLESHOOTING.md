# 故障排除与系统维护

## 构建问题排查

### 依赖检查流程
1. **检查系统依赖**:
   ```bash
   # 验证关键库是否安装
   dpkg -l | grep -E "libopencv|libeigen3|libyaml-cpp|libspdlog"
   ```

2. **检查动态链接**:
   ```bash
   # 检查可执行文件依赖
   ldd ./build/standard | grep -E "not found|undefined"
   ```

3. **检查SDK路径**:
   ```bash
   # 验证相机SDK是否正确安装
   ls -la /usr/lib/ | grep -E "MVSDK|MvCameraControl"
   ```

### 编译错误处理
1. **CMake配置错误**:
   - 错误现象: `find_package` 失败
   - 解决方案: 检查 `OpenVINO_DIR` 环境变量设置
   - 验证命令: `echo $OpenVINO_DIR`

2. **链接错误**:
   - 错误现象: `undefined reference`
   - 解决方案: 检查库路径和链接顺序
   - 验证命令: `make -C build/ VERBOSE=1`

3. **头文件错误**:
   - 错误现象: `fatal error: file not found`
   - 解决方案: 检查 include 目录设置
   - 验证命令: `cmake -B build --trace`

4. **ROS2 接口漂移 / 旧话题依赖失效**:
   - 错误现象: `src/sentry*.cpp` 或 `tests/*` 报 `io::ROS2` 没有 `subscribe_enemy_status`，或 `ros2.publish(Eigen::Vector4d&)` 无匹配重载。
   - 原因特征: 当前 `io::ROS2` 只保留了裁判系统发布和 `cmd_*` 订阅接口，而旧代码还依赖 `sp_msgs` 的 `enemy_status` / `autoaim_target` 链路；当前环境只有 `combat_rm_interfaces`，没有安装 `sp_msgs`。
   - 解决方案: 优先把调用点改到现有 `io::ROS2` API，移除失效的 `sp_msgs` 订阅/发布路径，不要直接把旧接口补回。
   - 验证命令: `make -C build -j2`

## 运行时问题排查

### 性能问题诊断
1. **帧率下降**:
   - 检查步骤:
     ```bash
     # 监控CPU使用率
     top -p $(pgrep -f standard)
     # 监控内存使用
     pmap $(pgrep -f standard) | tail -1
     ```

2. **延迟增加**:
   - 检查步骤:
     ```bash
     # 测量各阶段耗时
     ./build/standard_debug configs/example.yaml --profile
     ```

3. **通信延迟**:
   - 检查步骤:
     ```bash
     # 检查CAN总线状态
     ip -details link show can0
     # 检查串口权限
     ls -l /dev/ttyACM*
     ```

### 功能异常诊断
1. **相机无法打开**:
   - 可能原因: 相机被其他进程占用
   - 解决方案:
     ```bash
     # 检查相机占用
     lsof | grep /dev/video
     # 释放相机
     sudo kill -9 $(lsof -t /dev/video0)
     ```

2. **识别失败**:
   - 可能原因: 模型文件损坏或路径错误
   - 解决方案:
     ```bash
     # 验证模型文件
     file configs/example.yaml | grep "ASCII text"
     # 检查模型路径
     grep "model_path" configs/example.yaml
     ```

3. **控制异常**:
   - 可能原因: 通信协议不匹配
   - 解决方案:
     ```bash
     # 检查通信日志
     tail -f logs/*.screenlog | grep -E "send|recv"
     ```

4. **无桌面环境下 `imshow` 崩溃**:
   - 错误现象: 启动 `./build/auto_aim_debug_mpc` 或 `./build/sentry_omni_perception_debug_mpc` 后先看到 `XOpenDisplay Fail`，随后抛出 `Can't initialize GTK backend in function 'cvInitSystem'`。
   - 原因特征: 进程运行在无图形桌面、X11 不可达，或 `DISPLAY` 虽存在但当前用户无法连接，`cv::imshow` 第一次调用时会直接抛 `cv::Exception`；另外部分入口的 YOLO debug 绘制也会在内部触发 `imshow`。
   - 解决方案: `auto_aim_debug_mpc` 运行时可加 `--headless=true`，`sentry_omni_perception_debug_mpc` 默认不要加 `--display`；在已支持自动降级的版本里，首次 GUI 初始化失败后会自动关闭显示窗口，仅保留 Plotter/Dashboard/控制链路。
   - 验证命令: `./build/auto_aim_debug_mpc --headless=true`

## 标定问题排查

### 标定失败流程
1. **数据采集失败**:
   - 检查相机是否对准标定板
   - 检查光照条件是否合适
   - 检查标定板是否在视野内

2. **标定计算失败**:
   - 检查标定板角点检测
   - 检查图像质量（模糊、过曝）
   - 检查标定数据数量（至少15张）

3. **标定验证失败**:
   - 检查重投影误差是否过大
   - 检查标定结果是否合理
   - 检查坐标系定义是否一致

### 标定精度问题
1. **内参标定不准确**:
   - 解决方案: 增加标定图像数量（建议30-50张）
   - 优化方法: 使用不同角度和距离拍摄

2. **手眼标定不准确**:
   - 解决方案: 确保标定板位置固定
   - 优化方法: 使用机器人世界标定法

## 硬件问题排查

### 相机问题
1. **图像质量问题**:
   - 现象: 图像模糊、噪声大
   - 检查: 镜头焦距、光圈设置
   - 调整: 曝光时间、增益、伽马值

2. **同步问题**:
   - 现象: 图像与IMU数据不同步
   - 检查: 时间戳对齐机制
   - 调整: 延迟补偿参数

### 通信问题
1. **CAN总线问题**:
   - 现象: 数据丢失、通信中断
   - 检查: 总线负载、终端电阻
   - 调整: 通信频率、数据长度

2. **串口问题**:
   - 现象: 数据乱码、连接断开
   - 检查: 波特率、数据位、停止位
   - 调整: 流控设置、缓冲区大小

## 系统维护流程

### 日常检查清单
1. **硬件检查**:
   - 相机镜头清洁度
   - 连接线牢固性
   - 电源稳定性

2. **软件检查**:
   - 日志文件大小
   - 磁盘剩余空间
   - 系统更新时间

3. **性能检查**:
   - 帧率监控
   - 延迟测量
   - 资源使用率

### 定期维护任务
1. **每周维护**:
   - 清理日志文件
   - 更新系统软件包
   - 备份配置文件

2. **每月维护**:
   - 重新标定系统
   - 更新神经网络模型
   - 性能基准测试

3. **赛季维护**:
   - 全面硬件检查
   - 系统架构评估
   - 代码优化重构
