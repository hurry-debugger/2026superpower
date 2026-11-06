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

启动阶段先关闭四路功率输出，唯一匹配 UID 并拟合五路校准记录，然后完成两个 ADC 硬件
自校准。PowerCtrlTask_Init 准备控制、采样和通信后，一次启动 Master/A/B。
时基、中断请求、预装载及优先级由 CubeMX 配置和静态审查保证，启动时不重复核对。

ADC1/ADC2 使用 40/10 个半字的单块循环 DMA。ADC1 全满回调每 40 μs 直接读取活动数组，
完成十次整数近似平均、校准换算和 PowerCtrl_Task；允许平均期间窗口更新，不配对两路批次。
关闭两路 HT 和 ADC2 TC，保留 ADC1 TC 与两路 DMA 错误中断；真实采集错误锁存并标记反馈无效。
Master 每 1 ms 快照后发送两帧经典 CAN；Timer A/B 保留 PWM/ADC 触发，NVIC 关闭。
主循环为空闲循环，不实现采样停更检测或控制执行超期关断。

User/Procotol 统一容纳 USART BSP、Taurus 适配 CAN BSP、裁判 CRC/解析和超电协议。
USART1 的 IDLE IRQ 由 DMA 剩余计数取得长度，终止接收后直接调用裁判解析并重启 DMA。
0x0201/0x0202 完整通过 CRC、精确长度及结构体尺寸检查后，分别在短 PRIMASK 临界区
发布载荷和接收历史，闭环独立检查两类数据是否收到及年龄是否超过 300 ms。
其他命令及串口满缓冲、错误恢复流程保持现状，不将两类控制报文发布扩大解释为全部通信已验证。
CAN 0x100 发送电容电压与底盘电流，0x101 发送旧语义故障和状态；0x010 的首字节为1时
请求保护；所有输出由 PowerCtrl 在 ADC1 DMA 控制上下文唯一管理。

编译模式在 main.h 的 USER CODE Private defines 中二选一：默认 POWER_CTRL_CLOSED_LOOP，
执行完整四环及保护；POWER_CTRL_OPEN_LOOP 沿用 CubeMX 的固定标称 50% 互补窗口，
保留死区及半周期错相，首次 ADC1 TC 自动开启。开环不受校准、采样、PID 参数或裁判阻断，
仅检查 PWM 就绪、远程保护及状态合法性；远程解除后下一控制周期恢复。
开环必须外部断开 MOS 驱动供电或信号；软件开启 PWM 不能同时保证 MOS 不动作。

控制周期/采样周期/PWM 周期为 40/40/4 μs。CubeMX 配置已静态核对，PWM_READY=1；
PARAMETERS_READY=0、PID 系数及全零 UID/NAN 校准记录仍待填写，默认闭环禁止功率输出。
反馈、闭环、保护和通信均未完成实板验证，静态接入不能证明 40 μs 最坏执行预算。
裁判 0x0307 的源结构体105/常量103字节差异仍保留待核对，本轮不改变该命令处理。

## 目录

```text
2026superpower/
├── AGENTS.md             # AI 必读规则
├── README.md             # 工程入口
├── 2026superpower.ioc    # CubeMX 配置，禁止直接编辑
├── Core/                # CubeMX 生成代码，仅获批 USER CODE 内部可编辑
├── Drivers/             # 厂商驱动与 CMSIS
├── MDK-ARM/             # Keil 工程、启动与生成文件
├── User/                # Algorithm、Bsp、Calibration、Procotol 通信层、App 与 Config
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
