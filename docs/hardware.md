# 硬件资料与待确认事项

## 1. 已能从当前配置确认的信息

来源为当前 `2026superpower.ioc`、`Core/Inc/main.h` 和生成初始化代码的静态读取。
这里只记录配置事实，不代表实物连线、采样方向或输出波形已经测量确认。

芯片配置：STM32G474CETx。以下 GPIO 标签保留工程中的原拼写。

| 引脚 | 配置信号 | 工程标签 |
| --- | --- | --- |
| PA0 | ADC1_IN1 | `V_Chassis` |
| PA1 | ADC2_IN2 | `V_CUP` |
| PA2 | ADC1_IN3 | `V_BAT` |
| PB1 | ADC1_IN12 | `I_Chassis` |
| PB14 | ADC1_IN5 | `I_BAT` |
| PA8 | HRTIM1_CHA1 | 无用户标签 |
| PA9 | HRTIM1_CHA2 | 无用户标签 |
| PA10 | HRTIM1_CHB1 | 无用户标签 |
| PA11 | HRTIM1_CHB2 | 无用户标签 |

用户提供的原理图中 `V_CUP` 采样链对应 `VOUT_Cap`，代码枚举使用 `V_CAP`；
当前实板与图纸版本一致性仍待核对，生成 GPIO 标签不改名。
HRTIM 端口不能仅凭引脚表认定功率管的实际导通关系、驱动极性或 Buck/Boost 模式。

## 2. 本地资料索引

以下路径相对于工程根目录，属于本地参考资料；本轮换算依据见下一节，不能把全部资料均视为已审查。

| 本地路径 | 后续用途 |
| --- | --- |
| `references/PCB/Super-capacitor control.pdf` | 核对当前控制板原理图及其版本 |
| `references/PCB/ucc27200.pdf` | 核对驱动器规格、输入输出关系和时序约束 |
| `references/PCB/C730125_单片机(MCU-MPU-SOC)_STM32G474CET6_规格书_ST(意法半导体)单片机(MCU_MPU_SOC)规格书_merge.pdf` | 核对 MCU 引脚及电气条件 |
| `references/past_code/SuperCap2026_7_17/` | 旧工程采样、控制、保护与通信实现参考 |
| `references/港科开源/` | 开源方案参考，需要分别核对硬件与许可 |

`references/` 当前被 Git 忽略。新克隆缺少资料时标记“本地资料缺失”，再向用户确认获取方式；
不能把本地存在等同于仓库已包含，也不能将旧工程参数当作当前板卡已确认参数。

## 3. ADC 换算参数及来源

本轮依据用户指定的
`E:/RoboMaster/code/hardware/AD/Surpercap_control_MiniVerison1.5/Super-capacitor control.pdf`
第 3～5 页读取标称参数；不将本地 references 副本自动认定为同一版本。
宏定义位于 `User/Bsp/adc_convert.h`，基础参数与派生参数分开，均未完成实板标定。

| 基础宏 | 标称值及单位 | 来源 |
| --- | --- | --- |
| `ADC_REFERENCE_V` | 3.3 V | 第 4 页 U5 REF3033，ADC VREF+ |
| `ADC_FULL_SCALE_COUNT` | 4095 计数 | 当前 12 位右对齐配置，采用 ST LL ADC 换算宏的满量程映射 |
| `ADC_V_CHASSIS_DIVIDER_RATIO` | 14 V/V | 第 5 页 R26=13 kΩ、R34=1 kΩ，(13+1)/1 |
| `ADC_V_BAT_DIVIDER_RATIO` | 11 V/V | 第 5 页 R38=10 kΩ、R46=1 kΩ |
| `ADC_V_CAP_DIVIDER_RATIO` | 11 V/V | 第 5 页 R27=10 kΩ、R71=1 kΩ |
| `ADC_CURRENT_SENSE_GAIN` | 50 V/V | 第 5 页 U7/U8 INA240A2 及 TI 手册 |
| `ADC_CURRENT_SHUNT_OHM` | 0.003 Ω | 第 3 页 R3/R18 |
| `ADC_I_BAT_ZERO_V`、`ADC_I_CHASSIS_ZERO_V` | 各 1.65 V | 第 5 页 INA240 REF1 接地、REF2 接 U9 的 3.3V_A，标称居中 |

[TI INA240 官方手册](https://www.ti.com/lit/ds/symlink/ina240.pdf) 给出 A2 的 50 V/V 增益
及分接参考的输出关系。U9 的电流零点参考与 ADC U5 参考是两个器件；因此零点电压独立定义，
不将实测零点默认视为实际 ADC 参考的一半。

每个通道都有独立的 `ADC_<通道>_RATIO` 和 `ADC_<通道>_OFFSET_COUNT`：
电压比例为参考电压乘分压倍数再除满量程，零点为 0 计数；电流比例为参考电压除满量程、
增益及采样电阻，零点由独立零点电压换算，标称为 2047.5 计数，不提前取整。
通用计算为 `(raw_count - offset_count) * ratio`，保留负电流。
正电流对应采样电阻 INA240 IN+ 侧向 IN- 侧，实际端口正方向仍需确认。
满量程数值映射不代表放大器可达理想电源轨，也不代表消除了 ADC 量化和器件误差。

ADC1 Rank 顺序为 V_CHASSIS、V_BAT、I_BAT、I_CHASSIS，批量数据平均步长为 4；
ADC2 仅 V_CAP，步长为 1。当前 DMA 缓冲区仅为 4/1 元素，尚不保存 10 次扫描。

温度本轮不实现：原理图 R67/R68/R70 仅标为 NTC，缺少具体型号、R25、B 值及适用温区。
[Murata NTC 官方说明](https://www.murata.com/en-us/products/thermistor/ntc/overview/basic/characteristic)
说明了阻温特性和 B 常数；通用说明不能替代实际器件手册，也不能据此猜测本板参数。

## 4. 实现控制前需要确认

| 事项 | 当前状态 | 应补充的依据 |
| --- | --- | --- |
| 当前 PCB 版本及原理图匹配关系 | 待确认 | 板卡版本、对应原理图与修改记录 |
| 电压分压、电流采样增益与零点 | 图纸标称参数已记录，实板未标定 | 实板一致性、测量及标定数据 |
| ADC 参考电压、有效量程和采样时间 | 配置可读，实物与采样链路未验证 | 当前配置、测量和标定结果 |
| 电流正方向、功率符号与端口定义 | 待确认 | 电路路径与实际测量 |
| PWM 极性、死区、驱动关系与故障输入 | 待验证 | 配置解释、规格约束和示波器记录 |
| 保护阈值、迟滞、延时和恢复条件 | 待设计与批准 | 硬件额定值、需求及测试证据 |
| 控制周期、采样触发与执行预算 | 待设计与验证 | 时序方案与测量记录 |
| 通信协议、帧格式及超时策略 | 待确认 | 底盘接口要求与协议版本 |

获批补充时注明数据来源、单位、适用版本、日期及验证状态。
涉及外设配置时只能通过事先获批的 CubeMX 操作，禁止直接编辑 `.ioc`。
运行代码可在获批的 USER CODE 区域内部编写；区域外内容及标记行必须保持原样。
