# Repository Guidelines

**【最高优先级指令】**
`AGENTS.md` 只定义元规则与信息入口，不承载具体项目知识、参数细节或长篇规范。所有可变的项目内容统一从 `.agent/`、源码、`CMakeLists.txt` 与 `configs/` 获取，确保本文件始终精简、可稳定注入上下文。

---

## 阶段一：握手与加载 (Pre-task)
在处理任何开发、排障、重构或分析任务前，优先按以下顺序获取上下文：

1. `cat .agent/TODO.md`
   了解当前进度、待办项、已知问题和最近同步状态。
2. `cat .agent/DEVELOPMENT.md`
   加载编码规范、构建方式、命名约定和提交约束。
3. 按需读取 `.agent/KNOWLEDGE.md`
   当任务涉及架构、坐标系、MPC、EKF、多线程或通信链路时使用。
4. 按需读取 `.agent/TROUBLESHOOTING.md`
   当任务涉及编译失败、依赖错误、OpenVINO、ROS2、相机或硬件联调问题时使用。
5. 以仓库事实校验文档
   参数看 `configs/*.yaml`，构建目标看根和子目录 `CMakeLists.txt`，实现细节看 `src/`、`tasks/`、`io/`、`tools/`、`tests/`。

## 阶段二：执行约束 (Execution)
- 不要把具体知识回填到 `AGENTS.md`；稳定知识写入 `.agent/KNOWLEDGE.md`，流程规范写入 `.agent/DEVELOPMENT.md`。
- 所有开发、排障、构建、测试和文件改动必须在 `Combat_Sentry2026` Docker 容器内部完成。
- 不凭记忆猜测参数、目标名、脚本名或接口语义，必须回到 YAML、CMake 和源码核实。
- 当文档与实现不一致时，以当前源码和配置为准，再决定是否同步 `.agent/`。
- 读取遵循“最小充分”原则：先加载必要入口，再按任务扩展，避免无关上下文污染。

## 阶段三：收尾与状态同步 (Post-task)
完成任务后，检查是否需要同步 `.agent/`：

1. 进度或结论变化，更新 `.agent/TODO.md`。
2. 形成新的稳定架构知识，更新 `.agent/KNOWLEDGE.md`。
3. 沉淀新的报错特征或解决办法，更新 `.agent/TROUBLESHOOTING.md`。

除以上入口与路由说明外，不要继续扩写本文件。
