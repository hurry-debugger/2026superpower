# 2026 超级电容控制器

面向 STM32G474CETx 的 CubeMX / Keil 工程。本仓库同时保存生成代码、自维护规范和本地参考资料。

## 从这里开始

新对话或新 agent 先读 [AGENTS.md](AGENTS.md)，再读 [当前状态](docs/handoff.md)。
任何文件改动都要先在对话中列出详细清单，获得用户同意；Git 由用户人工提交。
**禁止直接编辑 `.ioc`；CubeMX 生成代码仅允许修改获批的 `USER CODE` 区域内部，
区域外内容及标记行必须保持原样。**

| 入口 | 用途 |
| --- | --- |
| [工程架构](docs/architecture.md) | 理解当前启动流程和未来代码边界 |
| [代码规范](docs/coding_style.md) | 编码、命名、排版与中文注释 |
| [AI 协作流程](docs/ai_workflow.md) | 修改前审批、验证与交接方式 |
| [硬件资料](docs/hardware.md) | 配置中可确认的引脚及待确认硬件信息 |
| [测试说明](docs/testing.md) | 检查命令、退出码与证据边界 |
| [工程决策](docs/decisions.md) | 已固定的约定及原因 |
| [用户代码目录](User/README.md) | 自维护固件的目录规划 |
| [代码模板](templates/module_template.h) | 明确需要状态返回时的特定示例；新模块默认返回计算结果或 void |

## 当前实现

依据当前文件静态读取：`Core/Src/main.c` 初始化 GPIO、DMA、HRTIM1 和 ADC1/ADC2，随后校准
两个 ADC，分别启动长度为 4 和 1 的循环 DMA，再调用 HRTIM 主计数器、A/B 定时器及对应输出
启动函数；主循环为空。新增的四次校准和 DMA 启动调用均检查返回值，失败进入 `Error_Handler()`。
`adc1_raw` 对应通道 1、3、5、12，`adc2_raw` 对应通道 2，仅保存原始计数供调试器观察。
`User/Algorithm/filter.c/.h` 已提供带步长的整数平均和内联浮点低通，
`User/Bsp/adc_convert.c/.h` 已提供通道枚举、参数宏、内联通用换算与数组转换。
调用顺序为整数平均 → `adc_raw[通道]` → `adc_convert[通道]` → 可选浮点低通，
调用示例见 [User/README.md](User/README.md)。这些模块及模板尚未接入 Keil 固件构建。
采样链路尚未上板验证，未实现功率闭环、保护状态机或温度换算。

## 目录

```text
2026superpower/
├── AGENTS.md             # AI 必读规则
├── README.md             # 工程入口
├── 2026superpower.ioc    # CubeMX 配置，禁止直接编辑
├── Core/                # CubeMX 生成代码，仅获批 USER CODE 内部可编辑
├── Drivers/             # 厂商驱动与 CMSIS
├── MDK-ARM/             # Keil 工程、启动与生成文件
├── User/                # Algorithm 滤波、Bsp 换算及调用说明，尚未接入固件
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
python -X utf8 -B -m unittest discover -s tools -p test_adc_feedback.py -v
```

ADC 数值测试需要已有 GCC，仅在系统临时目录编译测试程序；缺失时显示 `NOT_RUN`，
不能将跳过测试当作数值验证通过。安装环境中已有 `clang-format` 时，可以额外执行：

```powershell
python -X utf8 -B tools/check_project.py --format
```

普通检查只验证受管文本，不代表 C/H 排版、固件编译或上板测试通过。
缺少格式工具时，命令返回非零并报告 `NOT_RUN`。完整范围及行为见 [测试说明](docs/testing.md)。
