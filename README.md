# 2026 超级电容控制器

面向 STM32G474CETx 的 CubeMX / Keil 工程。本仓库同时保存生成代码、自维护规范和本地参考资料。

## 从这里开始

新对话或新 agent 先读 [AGENTS.md](AGENTS.md)，再读 [当前状态](docs/handoff.md)。
任何文件改动都要先在对话中列出详细清单，获得用户同意；Git 由用户人工提交。
**不得直接编辑 `.ioc` 或 CubeMX 生成代码，包括 `USER CODE` 区域。**

| 入口 | 用途 |
| --- | --- |
| [工程架构](docs/architecture.md) | 理解当前启动流程和未来代码边界 |
| [代码规范](docs/coding_style.md) | 编码、命名、排版与中文注释 |
| [AI 协作流程](docs/ai_workflow.md) | 修改前审批、验证与交接方式 |
| [硬件资料](docs/hardware.md) | 配置中可确认的引脚及待确认硬件信息 |
| [测试说明](docs/testing.md) | 检查命令、退出码与证据边界 |
| [工程决策](docs/decisions.md) | 已固定的约定及原因 |
| [用户代码目录](User/README.md) | 自维护固件的目录规划 |
| [代码模板](templates/module_template.h) | 新模块的接口、注释及排版参考 |

## 当前实现

依据当前文件静态读取：`Core/Src/main.c` 初始化 GPIO、DMA、HRTIM1 和 ADC1/ADC2，随后调用
HRTIM 主计数器、A/B 定时器及对应输出启动函数；主循环为空。
这不代表 ADC 采样链路、功率闭环或保护状态机已完成，也不代表已通过硬件验证。
`User/` 本轮只有目录说明；模板没有接入固件构建。

## 目录

```text
2026superpower/
├── AGENTS.md             # AI 必读规则
├── README.md             # 工程入口
├── 2026superpower.ioc    # CubeMX 配置，禁止直接编辑
├── Core/                # CubeMX 生成代码，禁止直接编辑
├── Drivers/             # 厂商驱动与 CMSIS
├── MDK-ARM/             # Keil 工程、启动与生成文件
├── User/                # 自维护固件规划，当前仅 README
├── docs/                # 架构、规范、状态、决策、硬件与测试
├── templates/           # 独立模块模板，不参与固件构建
├── tools/               # 只读规范检查及其测试
└── references/          # 本地旧代码、开源工程、硬件资料
```

`references/` 当前被 Git 忽略，新克隆工程不保证含有这些资料；协作规则不依赖该目录才能读取。

## 快速检查

在工程根目录使用 Python 3.10 或更新版本，无需第三方 Python 包：

```powershell
python -X utf8 -B tools/check_project.py
python -X utf8 -B tools/check_project.py --changed
python -X utf8 -B -m unittest discover -s tools -p test_check_project.py -v
```

安装环境中已有 `clang-format` 时，可以额外执行：

```powershell
python -X utf8 -B tools/check_project.py --format
```

普通检查只验证受管文本，不代表 C/H 排版、固件编译或上板测试通过。
缺少格式工具时，命令返回非零并报告 `NOT_RUN`。完整范围及行为见 [测试说明](docs/testing.md)。
