  ## Project Overview
  - 这是一个基于 C++ 和 CMake 的项目。
  - 先读 `README.md`。
  - 修改代码前先确认相关模块已有实现模式，不要凭空发明新结构。

  ## Repository Map
  - `src/`: 单元可执行文件源代码
  - `tasks/`: 功能层代码
  - `tests/`: 单元/集成测试
  - `docs/`: 架构、设计、开发文档
  - `logs/`: 运行日志
  - `configs/`: 运行时参数设置
  - `build/`: 构建文件
  - `assets/`: 模型等外部文件存放

  ## Build
  - 构建：` cmake -B build && make -C build/ -j`nproc` `
  - 不要自己发明新的构建入口；优先使用仓库现有脚本或 preset。

  ## Test
  - 修 bug 时，必须添加能复现该 bug 的测试。
  - 提交前至少跑与改动相关的测试。

  ## Coding Rules
  - 遵循现有代码风格，不要混入另一套命名/排版。
  - 不要为了“先跑通”而引入 mock 版本或简化组件，除非明确要求。
  - 优先修 root cause，不要堆 workaround。
  - 不要修改无关文件。
  - 不要提交生成物。

  ## Common Agent Failures And Corrections
  - 问题：只改代码，不补测试。
    修正：任何行为变化都要补测试；bugfix 必须有复现测试。
  - 问题：跳过 format/lint，导致 CI 失败。
    修正：提交前必须先跑 format/lint。
  - 问题：自己发明构建命令或绕过既有脚本。
    修正：只使用 README / CI / CMakePresets 里定义的入口。
  - 问题：用临时 workaround 掩盖真正问题。
    修正：先定位 root cause，再改实现。
  - 问题：误改 legacy 或无关目录。
    修正：仅改当前任务涉及的模块；legacy 目录除非明确要求不要动。
  - 问题：添加模型时未检查模型输入输出，导致推理模块出错。
    修正：在修改模型推理相关代码时，进行单元测试。

  ## PR Checklist
  - 已构建通过
  - 已运行相关测试
  - 已补测试
  - 已更新必要文档
  - 未提交生成文件
