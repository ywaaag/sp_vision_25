

---

#  核心开发规范与工作流 (DEVELOPMENT.md)

## 1. C++ 编程技术标准 (硬约束)

### 核心语言规范
- **标准**：C++17。
- **内存管理**：**禁止**使用 `new`/`delete`。必须使用 `std::unique_ptr` 或 `std::shared_ptr`。
- **并发编程**：
    - 必须使用项目内置的 `thread_safe_queue.hpp` 进行跨线程数据交换。
    - 互斥锁优先使用 `std::lock_guard` 或 `std::unique_lock` (RAII)。
- **现代特性**：优先使用 `auto` 推导复杂类型，使用 `enum class` 代替传统枚举，使用 `std::optional` 处理空返回值。
- **第三方库使用指引**：
    - **矩阵运算**：使用 Eigen3，禁止手写复杂的矩阵变换。
    - **日志**：统一使用 `spdlog`（通过项目 `logger.hpp` 接口），禁止使用 `std::cout`。
    - **视觉**：使用 OpenCV 4.x。

### 命名与代码风格 (遵循 `.clang-format`)
- **强制执行**：
    - 类名：`PascalCase` (如 `ArmorDetector`)。
    - 函数/变量：`snake_case` (如 `get_target_pos`)。
    - 常量/宏：`UPPER_SNAKE_CASE`。
    - 成员变量前缀：无特殊要求，但必须与同类文件保持一致。
- **格式化**：在提交前必须运行 `find . -name "*.hpp" -o -name "*.cpp" | xargs clang-format -i`。

## 2. 目录架构与文件职责
| 目录 | 职责 | 关键要求 |
| :--- | :--- | :--- |
| `io/` | 硬件驱动/协议 | 必须继承自 `CameraBase` 等抽象类，解耦硬件与算法。 |
| `tools/` | 通用工具 | 必须是高度复用的、不依赖业务逻辑的纯工具（如数学、日志）。 |
| `tasks/` | 核心算法 | 算法实现（如 `auto_aim`）必须支持独立于硬件的测试。 |
| `configs/` | 配置中心 | 所有魔法数字（Magic Number）必须写入 YAML，禁止在代码中硬编码。 |

## 3. 构建、测试与性能 (CLI 指令集)

### 常用命令快速参考
```bash
# 构建发布版本 (高性能)
cmake -B build -DCMAKE_BUILD_TYPE=Release && make -C build/ -j$(nproc)

# 构建并运行自瞄测试
make -C build/ auto_aim_test && ./build/auto_aim_test configs/example.yaml

# 运行标准 MPC 自瞄+打符
make -C build/ standard_mpc && ./build/standard_mpc configs/standard3.yaml

# 运行调试 MPC（JSON 数据记录 + 重投影可视化）
make -C build/ auto_aim_debug_mpc && ./build/auto_aim_debug_mpc configs/standard3.yaml

# 检查内存泄漏 (配合 Valgrind)
valgrind --leak-check=full ./build/auto_aim_test configs/example.yaml
```

### 质量红线
- **编译零警告**：必须开启 `-Wall -Wextra`，所有警告视为错误。
- **实时性要求**：在 NUC 环境下，主循环处理时间必须控制在 **10ms** 以内。
- **无硬件测试**：所有算法模块必须能在没有相机连接的情况下，通过读取 `recorder` 录制的视频数据完成回归测试。

## 4. Git 提交与版本控制 (Agent 行为守则)

### 提交信息模板
```text
<type>(<scope>): <subject>

[Optional body]
```
- **Type**: `feat` (新功能), `fix` (修复), `refactor` (重构), `perf` (性能优化), `docs` (文档)。
- **Subject**: 英文，动词开头，简述变更。

### 分支策略
- 严禁直接推送 `main`。
- 开发新功能时，必须先 `git checkout -b feature/xxx`。

## 5. 配置管理 (YAML)
- **参数解耦**：当新增算法参数时，必须在 `configs/example.yaml` 中同步添加对应的键值对并备注。
- **验证机制**：在 `yaml.hpp` 解析器中，必须对读入的参数进行有效范围检查（例如：曝光值不能为负）。

---
