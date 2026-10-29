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

依据当前文件静态读取：`Core/Src/main.c` 初始化 GPIO、DMA、HRTIM1 和 ADC1/ADC2，先禁用
四路功率输出，再调用无参 `UID_MatchCalibration()` 匹配记录并在启动阶段拟合五路系数。
随后执行两个 ADC 硬件自校准、4/1 元素循环 DMA 及 HRTIM 主/A/B 计数器启动，相关 HAL
返回值均检查；只有全局状态无板卡问题、五路拟合全部有效才启动四路功率输出。主循环为空。
`adc1_raw` 对应通道 1、3、5、12，`adc2_raw` 对应通道 2，仅保存原始计数供调试器观察。
`User/Algorithm/filter.c/.h` 已提供带步长的整数平均和浮点低通，
`User/Bsp/UID.c/.h` 提供完整 UID 匹配与全局 `g_adc_calibration_state`；
`adc_calibration_record.c/.h` 保存手填 UID 和二维采集数组；
`adccalibration.c/.h` 合并理论换算、启动拟合和 `AdcCalibration_Convert()` 数组转换。
调用顺序为整数平均 → `adc_raw[通道]` → `adc_convert[通道]` → 可选浮点低通，
调用示例和记录填写方法见 [User/README.md](User/README.md)。全部 C/H 已登记 Keil，
函数实现在 `.c` 中；模板不参与构建。当前仅有全零 UID、NAN 采集点的待填记录，因此
默认不启动功率输出。无记录或失败通道仍可用理论值观察，不能据此允许控制。
无需 Python/Excel 等校准工具；直接填写 C 记录、重新编译烧录，启动时自动计算系数。
尚未接入完整采样块、平均调度或反馈环，未实现运行期故障关断、功率闭环、温度换算，
采样链路及功率输出电气状态尚未上板验证。

## 目录

```text
2026superpower/
├── AGENTS.md             # AI 必读规则
├── README.md             # 工程入口
├── 2026superpower.ioc    # CubeMX 配置，禁止直接编辑
├── Core/                # CubeMX 生成代码，仅获批 USER CODE 内部可编辑
├── Drivers/             # 厂商驱动与 CMSIS
├── MDK-ARM/             # Keil 工程、启动与生成文件
├── User/                # Algorithm 滤波、Bsp UID/记录/校准及调用说明
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
```

默认仅做静态检查，无需新增测试脚本。既有专项测试保留，用户特别说明时按
[测试说明](docs/testing.md) 执行。安装环境中已有 `clang-format` 时，可以额外执行：

```powershell
python -X utf8 -B tools/check_project.py --format
```

普通检查只验证受管文本，不代表 C/H 排版、固件编译或上板测试通过。
缺少格式工具时，命令返回非零并报告 `NOT_RUN`。完整范围及行为见 [测试说明](docs/testing.md)。
