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

SuperPower_Init 先关闭四路输出，执行 ADC 硬件自校准，再识别 UID 并尝试逐通道拟合。
板卡信息中的 uncalibrated_channel_mask 按通道标记未校准状态，不生成控制错误；
未校准通道使用现有默认增益和零点偏置，成功通道使用拟合系数，均可参与闭环。
随后准备 DMA、UART、CAN，初始化控制并启动 Master/A/B；普通错误由 Master 更新。
固定外设配置由 CubeMX 和静态审查保证，不在业务代码中重复检查。

ADC1/ADC2 使用 40/10 个半字的循环 DMA。闭环 ADC1 TC 每 40 μs 直接读取活动数组，
完成十次整数近似平均、换算、底盘过流判断及四环控制；允许采样窗口更新，不配对批次。
短路仅为底盘电流绝对值严格超过 450 W / 15 V = 30 A，不附加电压或校准条件；
命中置 short_circuit、切 PROTECT，同周期关闭 PWM，锁存至 MCU 复位，禁止自动重试。
两路 HT 和 ADC2 TC 关闭，保留 ADC1 TC 和 DMA 错误中断；不实现停更或执行超期关断。

Master 每 1 ms 先检查普通错误，再快照发送两帧 CAN。电容、底盘共用 31 V 硬上限，
电容正常 (0,31] V、底盘有效 [15,31] V；电池数据须在 [15, ADC_V_BAT_MAX_V] V。
电池上界约 36.3 V 来自 ADC 理论量程，用于判断数据合理性，不是器件额定电压。
底盘在有效区间按 >30/>29/>28/>27 V 降序累加 125/25/5/1，达到 300 后饱和；
不高于 27 V 时清零，包括下电低压；跨档保留历史，不加入 12 V 电容限制。
普通故障重新赋值并可清除，短路锁存永不被慢速发布覆盖。

错误位为 cap_voltage、chassis_voltage、bat_voltage、referee、power_off、short_circuit，
电池电压与裁判主动下电各有唯一职责；未校准提示不参与错误字汇总。
慢速检查用两个短 PRIMASK 区完成三电压快照和错误字发布，判断在区外；
发布时合并最新短路位，只发布错误字；ADC 据此进入 PROTECT 并清零恢复计数。
ADC 唯一管理运行期 PWM、比较值、PID 和恢复。前馈所需电压不满足底盘 [15,31] V、
电容 (0,31] V 时，本周期直接保护、不进行除法、不设置普通错误位，详细归因仍由 1 kHz 完成。
有效电压时先算原始 Vcap/Vchassis，低于 0.05 同样保护且不另设电压错误位。
错误清零且前馈连续可用严格超过 12500 周期（约 500 ms）后复位四环并单次开启。
本周期还要求电池电压处于上述范围，两路电流处于各自 ADC 理论量程（约 ±27.5 A）；
以 `!(有效范围)` 统一拒绝异常，在恢复开启和 PID 前直接保护、清恢复计数，不新增错误位。
输入范围已保证两路功率乘积有界，不再重复检查乘积；30 A 短路锁存仍最先执行。

四环均每 40 μs 计算，不按串口新帧门控。入口功率 P_in=V_BAT*I_BAT，
A 端功率 P_A=V_CHASSIS*I_CHASSIS，能量环以 45 J 为目标输出修正 u_E（W）。
A 端目标为 P_lim-P_in+P_A-u_E，其中 P_lim 来自 robot_status.chassis_power_limit；
功率 PID 反馈同周期 P_A，误差等价于 P_lim-P_in-u_E，输出 A 端目标电流。
能量不足时正修正降低充电请求；入口采样未计入 R18 前板载自耗，计量偏差仍待实测。

电容平均目标参考上限改为 13 A，15 A 独立作为峰值设计约束，未实现峰值保护。
低于 5 V 充电参考上限 4 A、禁止主动请求放电；5～12 V 充电从 4 A 线性过渡到 13 A，
放电从 0 A 过渡到 13 A；达到 12 V 后双向 13 A。按电压比换算 A 端目标上下界，不引入效率参数。
电容高于 26.1 V 时，29 V 目标的电压 PI 与电流 PID 竞争同一前馈上的占空比，取较小值；
四环 Improve 均为 NONE，采用基础位置式 PID，不执行可选积分限幅、抗饱和、滤波或输出跟踪。
算法层保留这些可选实现供后续调试；控制层仅计算、选择及限幅，不处理积分回算或撤销。
NONE 模式只限制最终输出，不启用积分限幅；零误差时 Output=0，Iout 保留。
13 A 只约束目标，电压覆盖期间实际电流及 15 A 峰值不能由此保证；校准范围独立保持 15 A。
校准采集点和 PID 参数仍待填写；未校准不单独禁止闭环，其他故障和恢复条件仍生效。
调试器查看 g_board_info.uncalibrated_channel_mask：0 为全部已校准，非零表示仍有未校准通道。
当前源码没有 PARAMETERS_READY/PWM_READY 门槛宏，不把旧说明当作已实现的保护。

超电控制板可接收的现有 0x0201 为 17 字节，保留 bullet_speed_limit；0x0202 为 14 字节，
原功率位置仍是保留字段，不用作实际功率反馈。
USART1 IDLE IRQ 直接解析并重启 DMA。两类裁判命令共用 g_judge_rx_update_ms，
时间戳在 CRC 校验前刷新，载荷通过 CRC 和长度检查后复制；300 ms 超时沿用此时间戳，
不能等同于完整的有效报文检测。裁判关闭底盘输出由 power_off 表示。
CAN 0x100 发送电容电压/底盘电流，0x101 显式映射旧故障位 0/2/3/7 和状态 0/1；
0x010 首字节仅记录，开闭环均不受 CAN 模式命令控制。

模式在 main.h 的 USER CODE 区二选一。OPEN_LOOP 保留 CubeMX 固定标称 50% 互补窗口，
当前 ADC1 TC 每次直接调用输出开启，不处理采样或运行控制；Master 仍更新错误字并发送 CAN，
但开环不执行 PowerCtrl_Task，错误字不能据此关断 PWM。本轮未改该既有调度。
开环必须外部隔离 MOS 驱动，其 CAN 测量保持初始化值。模式分支位于调度层，
power_ctrl.c/.h 不按模式裁剪；不修改生成文件或外设配置。

以上均为静态实现事实，未完成 ARMCC 固件构建或实板验证。
30 A 高于标称理想采样范围 ±27.5 A，可检测性、关断效果及最坏 40 μs 执行时间待测。

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
