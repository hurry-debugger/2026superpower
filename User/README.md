# 自维护固件目录

当前已有滤波、UID/校准、HRTIM、BSP 数据发布及四环控制模块，均已登记 Keil。
采样、裁判接收和控制已接入调度；当前校准模板无效，禁止闭环输出，PID参数仍待填写。
启动保留UID匹配及拟合，ADC1 DMA接入近似平均、过流和控制；Master检查普通错误并发送CAN。
不启用可选低通；上板验证待完成。
新增代码前必须先阅读根目录 `AGENTS.md` 与 `docs/coding_style.md`，在对话中提交逐文件修改清单，
获得用户同意后再创建目录、代码或修改构建配置。

## 模块与规划

```text
User/
├── App/          # 应用状态、任务编排、保护策略与业务协作
├── Bsp/          # UID、HRTIM 与采样发布
├── Calibration/  # 校准记录、拟合算法及理论换算
├── Procotol/     # USART/CAN BSP、超电协议、裁判类型、CRC 与解析（沿用拼写）
├── Algorithm/    # 已有 filter.c/.h：整数平均、浮点低通
└── Config/       # 有明确单位、来源和适用范围的配置
```

App、Config、Calibration、Procotol 已登记构建；校准记录 C/H 从 Bsp 移至 Calibration，
内容与原有 AdcCalibration 接口不变。
C/H 在模块目录中配对存放，文件名小写下划线；UID.c/.h 为用户指定命名例外。
新模块可以从 `templates/module_template.c/.h`
提炼结构，但其中的状态接口仅供明确需要状态返回的场景；简单计算直接返回数值或使用 void。

App 可依赖 Bsp、Algorithm、Config 和 Procotol；底层模块不反向包含 App。
Algorithm 不依赖 HAL；USART/CAN 的 HAL 封装位于 Procotol，Bsp 管理采样与功率输出。

## 手填记录与启动校准

只需在 `main` 的启动区域调用一次 `UID_MatchCalibration()`，无参数、无返回值。
它在 `UID.c` 中读取完整 UID，唯一匹配记录，调用 `AdcCalibration_Init(id)` 并发布全局状态。
`g_adc_calibration_state` 唯一定义在 UID.c，extern 与完整类型在 adccalibration.h 中声明，
其他模块包含该头文件后只读；不能自行修改问题位或有效掩码来允许控制。

校准流程不需要脚本或 Excel：

1. 查看全局状态的 `uid.word[0..2]`，按相同字序填写记录中的三个 uint32_t。
2. 在 `adc_calibration_record.h` 增加枚举 ID，放在 `SUPERCAP_COUNT` 之前；
   在对应 .c 数组使用 `[SUPERCAP_ID_XX]` 指定下标。01 标签对应下标 0。
3. 每路按 `ADC_CAL_RAW_ROW(通道)` 填平均 ADC 计数，允许小数；按
   `ADC_CAL_REAL_ROW(通道)` 填仪表实测 V/A，同列为一组。计数定义应与实际采集平均一致，
   不先用理论公式换成 V/A；电流保留正负号。
4. 默认每路固定 5 点，缺失位置保留显式 `NAN`；计数和实测值均可能合法为零。
   C 会把省略的元素补零，程序无法将其与真实零值区分，因此必须保留完整模板逐项替换。
   调整 `ADC_CAL_POINT_COUNT` 后同步填写每路全部位置，不使用零方差数据。
5. 重新编译、烧录同一固件；启动时以中心化最小二乘计算 `gain`、`bias`，之后不再拟合。

记录结构只有 UID 和 `[5通道 * 2行][固定点数]` 数组两个成员。全零 UID 槽位不参与匹配。
当前模板没有真实 UID 或采集数据，默认不启动功率输出。更换 MCU 后重新绑定，更换采样电阻、
放大器等模拟前端后重新校准。拟合后的数值校验不等于实板精度验收；不预设残差合格阈值。

无记录或重复匹配使用理论值供观察；有记录时有效通道缓存拟合系数，失败通道保留理论值。
全局状态记录当前 UID、超电 ID、板卡错误位域、每通道错误位域及有效通道掩码。
原始计数的 NAN/无穷归入 count_range，实测值的 NAN/无穷归入 value_range；
已删除恒零的 missing_point、nonfinite 字段，通道错误仍占一个32位字。每路首个错误停止拟合，不静默丢弃坏点。
负 Bias 合法。启动控制只读取全通道有效掩码是否为 `ADC_CAL_ALL_CHANNELS_MASK`，
由此生成一次 calibration 汇总；未知/重复 UID、非法 ID 或任一拟合失败均无法得到完整掩码，
详细板卡/通道原因留在校准模块。底盘、电容校准共用31 V，两路电流校准使用独立的
ADC_CAL_CURRENT_MAX_A=15 A，不随13 A平均目标参考上限改变；
电池实测值只要求非负有限，没有43 V或其他物理上限，计数、正增益及可表示性检查保留。

main 先禁用输出，再匹配拟合；保留 ADC 硬件自校准、DMA 和采样触发计数器启动。
这是启动门槛，不包含运行期测量故障关断。再次匹配或拟合只能在功率输出关闭、消费者尚未
运行时进行；不能与中断换算并发。初始化后状态和系数保持只读，不默认使用 volatile。

## 平均后转换

实际采集位于Bsp/bsp_adc.c：ADC1[40]为十次四通道扫描，ADC2[10]为十次单通道采样。
两路循环DMA继续写入；CPU通过volatile读取，按ADC1步长4、ADC2步长1累加，
使用(sum + 10 / 2) / 10四舍五入为五路uint16_t计数，再调用AdcCalibration_Convert。
不把活动数组强转后交给要求稳定输入的Filter_AverageU16，不复制稳定槽、不配对批次。
不等待首轮填满，允许读取期间窗口更新和两路窗口交叠，结果是近似十次平均。

Process直接使用g_bsp_adc_feedback中的adc_sum和adc_raw，不再定义局部求和变量或均值数组。
一个uint32_t adc_sum供ADC1四通道与ADC2顺序复用，每通道开始前清零；
五路uint16_t adc_raw全部写入后，统一换算到adc_convert，电压单位V、电流单位A。
工作区仅在不可重入的ADC1 TC采样上下文使用；无valid、发布时间或发布序号。
接口头文件为Bsp/bsp_adc.h，函数、类型及全局对象名称保持原样。
采集层不判断数值有效性；控制层保留原有数值与物理保护判断，不读取采集错误。
反馈不表示校准有效，也不承诺固定批次一致。
闭环ADC1 TC随后同上下文调用PowerCtrl_Task；开环跳过处理，仅开启固定PWM；Master读取时取得一致快照。

调用接口为：

```c
BspAdcFeedback_Init();    /* 启动阶段，计数器未运行、输出关闭。 */
BspAdcFeedback_Process(); /* 仅闭环ADC1全满回调中，随后调用PowerCtrl_Task。 */
```

AdcCalibration_Convert返回输出数组地址，不代表校准成功；无有效校准时结果仅供观察。
统一公式为raw_count * gain + bias，拟合偏置直接来自实测数据，不再扣一次零点。
AdcCalibration_ConvertValue的第三参数单位为V/A，与旧计数零点接口不同。

## 可选浮点低通

默认直接使用 `adc_convert` 作为反馈。需要低通时，在换算后执行下列示例，
由调用方长期保存 `adc_lowpass`，以其作为滤波后的反馈。不同通道不共用历史值。
首次传入 `first_block = true`，之后传入 false；重新开始采集或丢失历史时重新初始化。
`alpha` 由调用方按实际反馈周期选定，范围为 [0, 1]；这里不指定截止频率或默认系数。

```c
#include "filter.h"
#include "adccalibration.h"
#include <stdbool.h>

/**
 * @brief 更新调用方保存的各通道物理量历史，首次直接使用当前换算值。
 * @pre 两个数组均有 ADC_CHANNEL_COUNT 个元素，alpha 有效；非首次时历史已初始化。
 * @note 固定反馈周期调用；本示例不在固件默认采样路径中。
 */
void Feedback_LowPass(const float adc_convert[ADC_CHANNEL_COUNT],
                      float adc_lowpass[ADC_CHANNEL_COUNT], float alpha, bool first_block)
{
    for(uint32_t channel = 0U; channel < ADC_CHANNEL_COUNT; channel++)
    {
        if(first_block)
        {
            adc_lowpass[channel] = adc_convert[channel];
        }
        else
        {
            adc_lowpass[channel] = Filter_LowPass(adc_convert[channel], adc_lowpass[channel], alpha);
        }
    }
}
```

如需功率，由应用层按对应端口的电压与电流计算，不增加 ADC 功率通道。
低通会增加反馈延迟；实际系数、内联展开及 40 μs 执行预算须由目标编译器和上板测量确认。
filter、Bsp/UID、Calibration和运行模块均已登记Keil。采集仅保留两块DMA数组。
启动后关闭两路HT和ADC2 TC，保留ADC1 TC及两路DMA错误中断。
不实现采集故障锁存、自定义HAL错误回调或OVR读取；HAL启动失败仍进入Error_Handler。
采集错误不接入控制保护；没有自动重启、停更或执行超期检测。

## HRTIM 广义占空比底层模块

`Bsp/bsp_hrtim.c/.h` 已提供四个 `void` 接口并登记 Keil；由 PowerCtrlTask 启动，
闭环由 PowerCtrl 管理输出，开环 ADC1 回调当前每次直接调用开启；运行期均在 ADC1 DMA 上下文。
当前 CubeMX 已开启 A/B 预装载和复位更新；波形、死区、自举刷新仍待上板验证。

| 接口 | 职责与限制 |
| --- | --- |
| `BspHrtim_Init()` | 关闭四路输出，恢复非中间模式；不启动计数器、不改外设配置和比较值 |
| `BspHrtim_SetGeneralizedDuty(g)` | 限幅、模式迟滞、占空比分配和 CMP1/CMP2 预装载写入；不自动开启输出 |
| `BspHrtim_EnableOutput()` | 统一开启 TA1/TA2/TB1/TB2；允许开启的依据由调用方检查 |
| `BspHrtim_DisableOutput()` | 统一关闭四路输出；保留计数器、比较值和迟滞状态 |

`g` 为理想电容侧电压/底盘侧电压，无单位，允许大于 1，必须是有限正数，
随后限幅到 0.05～10。零、负数、NaN 和无穷不属于支持输入，关断须使用独立接口。
这不是电流方向命令；应用层负责闭环、反馈有效性和校准门槛，Bsp 不读取 ADC 状态。
启停 HAL 返回值失败时进入既有 Error_Handler()；接口不新增状态返回值。

本板原理图第 3 页确定 Timer A 接电容侧、Timer B 接底盘侧；CH1 均接下管，CH2 均接上管。
定义上管占空比 `D_chassis` 和 `D_cap`，理想比例为 `g = D_chassis / D_cap`。
中间模式使用 `D_chassis = 0.4 * (g + 1)`、`D_cap = 0.4 * (1 / g + 1)`；
其余模式按下表分配：

| 条件 | 底盘侧上管 D_chassis | 电容侧上管 D_cap |
| --- | --- | --- |
| g < 1 | 0.9 * g | 0.9 |
| g >= 1 | 0.9 | 0.9 / g |

初始为非中间模式，在 `0.9 < g < 1.111` 时进入中间模式；在 `g < 0.8` 或 `g > 1.25`
时退出。进入边界等于 0.9/1.111 时不进入，退出边界等于 0.8/1.25 时仍保留中间模式。
完整分配策略和全部数值来源是港科 RM2024 配套 `PowerManager.cpp` 的 `updatePWM()`；
进入阈值 0.9/1.111 来自源码，不能称为开源报告正文提供的参数。
它们是用户批准沿用的参考初值，不是本板已验证的额定范围、保护阈值或最小脉宽。

CH1 的中心窗口是下管脉宽，故 `CMP1 = round(P * D_upper / 2)`、`CMP2 = P - CMP1`；
Timer A 采用 D_cap，Timer B 采用 D_chassis，周期分别读取当前周期寄存器。
先在局部算完四个值，再临时禁止 A/B 预装载传输、完整写入并恢复原更新禁止位。
其他 CR1 位保持原值；CMP3、输出事件来源、周期、死区、相位及计数器均不改写。
这里的上管占空比是理想值，未补偿正死区、驱动延迟与计数量化，不保证实测电压比。

### CubeMX 配置与运行前提

以下为用户本轮通过 CubeMX 生成后核对的要求；代理不直接改写生成配置：

- Timer A/B：`PreloadEnable = HRTIM_PRELOAD_ENABLED`、
  `ResetUpdate = HRTIM_TIMUPDATEONRESET_ENABLED`、
  `UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT`。
- Timer A/B：`UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE`、
  `RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED`；不引入其他自动更新来源，
  调用方不触发 A/B 软件更新。由各自复位/回卷事件装载新值。
- 主/A/B 保持相同且固定的周期、向上连续计数。当前周期为 21760 计数，
  A 由主周期复位，B 由主 CMP1=10880 复位，保留半周期错相。
- 保留 CH1 在 CMP1 置位、CMP2 复位，CH2 正死区互补、四路高有效及 CMP3 ADC 触发。
  当前正死区升降沿值均为 15，仍须核对器件、脉宽及实板波形，不能据此声称足够。

两侧分别在下一次自身复位/回卷时生效，相差半周期；不承诺 A/B 同时切换。
只有一个调用上下文持有 A/B 比较值、更新禁止位及模块私有模式状态，调用不可重入，
不与其他 HRTIM 公共寄存器或 HAL 句柄写入并发；运行期两次设置至少间隔一个完整 PWM 周期。
临时禁止装载不是全局临界区，也不提供任意中断抢占关断的故障保护能力。

运行顺序为：先关闭输出、完成 UID/拟合及 ADC 硬件校准，再 PowerCtrlTask_Init，
其中 PowerCtrl_Init 调用 BspHrtim_Init，准备 DMA/通信后一次启动三计数器。
闭环启动先设置校准汇总并检查零初始反馈；PROTECT 每周期关断并预置前馈，
普通错误由1 kHz更新，ADC在错误字为零时累计连续正常控制周期，
严格超过 500 ms 对应的 12500 次后清零计数并开启输出，不另设装载等待阶段。
main 不直接开启四路输出。此处是源码调用关系，非硬件运行记录。

## 与生成工程的边界

**禁止直接编辑 `.ioc`；CubeMX 生成代码仅允许修改获批的 `USER CODE BEGIN/END` 区域内部，
区域外内容及标记行保持原样。** 自维护业务模块优先放在 `User/`。
放在 User 中的生成文件仍受同样保护，不能通过移动代码规避边界。
启动和回调可在对应的 USER CODE 区域内接入，仍须先列出具体修改清单并获批。
后续新增或重命名 C/H 必须把 Keil 分组与头文件路径维护纳入同一清单；
本目录说明不单独授权写文件或添加运行时调用。

## 应用层控制与运行接入

`App/power_ctrl.c/.h` 提供 `PowerCtrl_Init(void)`、`PowerCtrl_Task(void)`、`PowerCtrl_UpdateErrors(void)`，
公共实现依次为Init、Task、UpdateErrors，私有ModeSwitch/DutyCalcu执行状态及输出，闭环计算封装在CascadePidCalcu。Init内按Taurus形式逐环调用
PID_Init，先清零实例；死区为0，不启用电机堵转检测。
能量、功率、电流、电压四环Improve均为NONE，不执行任何可选优化，死区内输出为0。
算法层保留积分限幅、外部抗饱和、微分/输出滤波、梯形/变速积分及输出跟踪等已有实现，
调试时由用户显式启用；增量式Ki=0清零行为保持。Proportional_On_Measurement仍只有原选项位，
没有对应实现，不将其描述为已实现功能。

### 编译模式与配置

在 `Core/Inc/main.h` 的 USER CODE Private defines 中二选一：

| 宏 | 行为 |
| --- | --- |
| POWER_CTRL_CLOSED_LOOP | 默认；ADC采样、底盘过流和四环，Master普通保护及CAN |
| POWER_CTRL_OPEN_LOOP | 固定标称50%波形；ADC不运行Process/Task，Master仍更新错误字；须外部隔离驱动 |

main.h保留未选/双选检查。现有模式分支位于调度层启动和ADC回调，Master当前不分支；
power_ctrl.c/.h保留完整闭环实现，不增加运行期模式或CAN使能。
配置集中在 `Config/power_ctrl_config.h`。当前没有PWM_READY/PARAMETERS_READY宏；
PID系数和真实UID/校准记录仍待填写，默认校准失败禁止闭环输出。
开环测量保持初值，Master仍可更新错误字，但开环ADC不读取错误字来关断，必须外部隔离MOS驱动。

### 数据与调用上下文

闭环ADC1 DMA全满回调以优先级0先执行BspAdcFeedback_Process，再执行PowerCtrl_Task。
高速先检测底盘过流，再检查前馈电压和可达性、读取普通错误，决定状态、PWM及PID。
前馈无效时只直接保护和清零恢复计数，普通错误归因仍在1 kHz。
Master优先级1，每1 ms调用PowerCtrl_UpdateErrors，再调用SupercapCan_SendStatus。
Master只发布错误字，不改PWM/PID；ADC下一周期据错误字保护并清恢复计数。UART优先级3。

慢速检查的第一个短PRIMASK区复制三个电压和错误字，判断与分档计算在区外。
发布时第二个短临界区合并当前short_circuit，避免ADC刚置位后被旧快照覆盖；
只发布合并后的错误字，使用原PRIMASK恢复屏蔽状态；状态和恢复计数由ADC处理。
三电压、错误字及PRIMASK是必要临时快照，不复用仍由ADC读写的控制记录，不新增永久副本。
state/errors/errclearcnt为中断共享volatile字段；原子性依靠临界区，不能只靠volatile。
底盘加权累计只由Master更新。CAN另取自己的最小一致快照，编码和发送在区外。

开环ADC每次直接调用固定PWM开启，跳过Process/Task；Master仍检查普通错误并发送CAN。
该开环路径不据错误字关断，不具备闭环软件保护；本轮仅修正接口注释，调度行为保持不变。
不保存采样序号、年龄或执行超期状态，采样DMA继续运行但CAN反馈保持初始化值。
Timer A/B关闭NVIC业务中断，保留PWM与ADC触发。

裁判仍由USART回调直接解析，两类控制命令共用g_judge_rx_update_ms且在CRC前刷新；
普通保护只据此检查300 ms超时。power_off仅表示机器人状态中的底盘输出位为0。

### 裁判协议与仅接收 USART

超电控制板可接收的现有0x0201载荷为17字节，保留bullet_speed_limit；0x0202为14字节。
本轮不改变协议布局或来源说明，reserved3仍为保留字段，入口功率由本板ADC计算。
当前解析只接收 0x0201/0x0202，载荷在 CRC8、CRC16 和长度检查后复制到对应全局对象。
现有复制不提供旧说明中的两类 valid/update_ms 或短 PRIMASK 原子发布；本轮不修改通信代码。
底盘输出位来自机器人状态载荷；本轮不另存副本、不新增接收状态或改变协议布局。
接收超时、半帧、载荷一致性及 ARMCC 位域布局仍待另行核对，不能将当前接入解释为通信已完整验证。

Procotol/bsp_usart.c/.h 绑定已生成的 USART1 / DMA1_Channel3 普通 RX DMA。
USER_UART_Init(void) 保留空闲回调、中断、初始化结构；USART IRQ 在原 HAL IRQ 前处理 IDLE，
以 RxXferSize 减 DMA 剩余计数取得长度，AbortReceive 后直接解析并重新启动 DMA。
现有错误回调停止 DMA 后重启；接收返回值、满缓冲及接收故障恢复仍待另行核对，
本轮不增加重试、半帧重组或采集故障机制，也不修改生成代码中的串口入口。

### 闭环控制关系

- 四环每40 us连续计算，PID系数按调用定义，不按新报文门控或自动补偿裁判更新周期。
  能量位置式PID以45 J减裁判缓冲能量形成误差，Output是功率修正u_E，单位W。
- 入口功率P_in=V_BAT*I_BAT，A端反馈P_A=V_CHASSIS*I_CHASSIS，正值充电、负值放电。
  A端目标P_A,target=P_lim-P_in+P_A-u_E，P_lim来自robot_status.chassis_power_limit。
  加回P_A相当于移除入口中已有的超电支路贡献，再计算允许的A端功率；
  以同周期P_A为反馈，功率PID误差化简为P_lim-P_in-u_E，输出A端目标电流A。
  正向能量增益下能量不足产生正修正，减去它可减少充电或增加补电；增益仍待整定。
- ModeSwitch先检查前馈所需电压、V_BAT在[15, ADC_V_BAT_MAX_V]及两路电流各自的ADC理论量程。
  以`!(有效范围)`拒绝NaN、无穷和极端有限值，异常在开启和PID前直接保护、清恢复计数。
  电池上界约36.3 V，两路电流约±27.5 A，仅用于数据合理性，不代表器件额定值或采样精度。
  有效输入保证|P_A|不超过约853 W、|P_in|不超过约999 W，删除重复乘积检查，不新增错误位。
  power_feedback_w存同周期P_A，power_target_w先存P_in，在CTRL内原位形成A端目标。
  恢复复位保留这些已检查的工作值，PROTECT期间不能把power_target_w当作有效控制目标。
  不增加功率副本、状态、按帧标志或积分快照，普通错误发布仍由1 kHz归因。
- B端平均目标参考：Vcap<5 V时充电4 A、放电0 A；
  5≤Vcap<12 V时令x=(Vcap-5)/7，充电4+9*x A、放电13*x A；
  Vcap≥12 V时双向13 A。5 V和12 V两端连续，12 V不是硬欠压门槛。
- A端上下界分别为B端充电参考乘Vcap/Vchassis、负的B端放电参考乘Vcap/Vchassis。
  不引入效率参数；两个current_*_limit_a成员先作为B端安培工作值，随即原位换算。
- 电流PID和29 V目标电压PI都输出无量纲修正u，候选为Dff+u；
  Vcap>26.1 V取较小候选，否则只采用电流候选。删除电压PID输出正向电流上限的旧链。
  两个内环每周期均正常计算，未被选中的环不跟踪、不冻结也不撤销积分。
- 四环使用NONE：无可选积分限幅、条件积分抗饱和、输出跟踪或滤波。
  控制层无积分快照、积分预限幅、回算和撤销；目标电流边界、各级输出限幅和保护仍保留。
  零误差死区输出为0，累计积分仍保存；不承诺退出饱和或控制权切换无跳变。
- 算法层普通/受限位置式接口共用单次计算核心。外部抗饱和仅在受限接口且选项启用时生效，
  只保存一个旧积分浮点快照，拒绝同向积分后仅重合成输出，不重算P/D、不修改Ki或推进第二次历史。
  内部积分限幅原有流程保留，恢复旧积分仍服从已启用的IntegralLimit；输出滤波重合成使用同一Last_Output。
- PID_TrackOutput保留在算法层，用实际采用值减去本周期P/D回算积分并按IntegralLimit约束；
  当前控制层没有调用。以后接入环间跟踪需由调用方传入实际采用值，不能只修改选项就自动获得控制权信息。
  跟踪实例仍要求关闭OutputFilter，积分范围不足时不保证完全无扰；40 us预算仍待目标测量。
- 复用generalized_duty：ModeSwitch先保存原始前馈，最后叠加被采用修正并限幅到0.05～10。
  候选使用两个PID的Output，不增加占空比副本、竞争状态标志或低压PID。
- 13 A只约束目标；电压覆盖可能使实际A端电流更负，纯min竞争不能保证实际B端平均电流。
  15 A峰值仅记录在POWER_CTRL_CAP_CURRENT_PEAK_A，未实现B端峰值检测或保护。
- App只调用现有BspHrtim接口，逻辑A/chassis对应本板Timer B，电容对应Timer A；
  PWM频率、桥臂映射、ADC触发、DMA和中断优先级均保持。
- P_in不含R18前板载供电自耗，不能直接视为裁判计量值；接线、采样方向及动态误差待实测。

### 两状态与故障时间

运行状态仍为PROTECT=0、CTRL=1，CAN显式映射NORMAL=0、PROTECT=1。
恢复errclearcnt只在PROTECT、错误字为零且前馈可用时按ADC周期递增，第12500次保持保护，
第12501次清零、复位四环并单次开启。ADC读到错误、检测过流或前馈不可用均清零等待；
持续CTRL不反复开启，PROTECT不运行PID，只关断；有效电压时更新并预置前馈，无效时保留旧记录。

| 内部标志 | 条件及频率 | 解除方式 |
| --- | --- | --- |
| cap_voltage | 电容不满足(0,31] V，含NaN/无穷；1 kHz | 正常后清零 |
| chassis_voltage | 底盘不满足[15,31] V，或分级累计>=300；1 kHz | 条件解除后清零 |
| bat_voltage | 电池不满足[15, ADC_V_BAT_MAX_V] V，含无效数值；1 kHz | 正常后清零 |
| power_off | 裁判底盘输出位为0；1 kHz | 恢复许可后清零 |
| referee | 共用接收时间戳年龄>300 ms；1 kHz | 超时解除后清零 |
| calibration | 全通道有效掩码不完整；启动汇总一次 | 运行期不变 |
| short_circuit | 仅abs(I_CHASSIS)>30 A；25 kHz | 锁存至MCU复位，不自动重启 |

1 kHz分级累计使用同一个chassis_overvoltage_cnt，不另存档位/步长。
有效底盘电压按>30/+125、>29/+25、>28/+5、>27/+1降序判断，>=300饱和；
跨档保留，不高于27 V时清零，包括低于15 V的下电。固定档从零分别需3/12/60/300次检查，
名义约3/12/60/300 ms。31 V本身属于最高累计档，严格超过31 V才立即置硬过压；
15 V本身有效。无效电压仍保护，NaN不满足累计清零条件。
电容31 V有效，零电压无效，不增加12 V限制；电池15 V和ADC理论量程上界均有效，不恢复43 V上限。
控制路径禁止与FLT_MAX比较，使用上述有效范围；校准中的旧可表示性比较未在本轮修改。
普通故障解除后仍须等待约500 ms，短路锁存始终阻止恢复。

普通错误由Master发布错误字，PWM由下一次ADC控制关断；短路和前馈失效在本次ADC控制关断。
ModeSwitch在短路判断后先确认底盘[15,31] V、电容(0,31] V，再计算原始Vcap/Vchassis。
电压无效不进行除法、不写零前馈、不改普通错误位，直接PROTECT并清恢复计数。
原始比值低于0.05也直接保护，但不设置cap_voltage/chassis_voltage；24 V底盘下电容不足1.2 V不能启动。
这是可达性必要条件，不是已测最小有效脉宽；不足区域需外部预充，本轮不提供预充流程。
没有参数就绪门槛、采样误差阈值或驱动电源监测，不把缺少反馈的条件伪装成已实现保护。
短路不检查电压和校准；正负无穷电流会触发绝对值比较，NaN不会触发，不新增通用数值位。
开环不执行这些保护。删除电容/电池连续计数和旧补满逻辑，只保留底盘加权与恢复两类历史。

13 A平均目标、15 A峰值设计约束和独立15 A校准范围分开管理；ADC换算保持。
底盘关断阈值仍由450 W/15 V派生为30 A。
30 A高于标称理想采样范围±27.5 A，可检测性、关断及防反灌待实板验证。
恢复时长取决于实际ADC回调频率，超期或事件合并会改变物理等待时间；
裁判超时使用HAL_GetTick与共用接收时间戳，不建立另一份恢复时钟。

### 参考工程计时核查（静态配置，不是实板测量）

港科 RM2024 `Core/Src/main.c:133-150`：8 MHz HSE /1 *9=72 MHz，APB1 /2，
定时器时钟乘2回到72 MHz；`Core/Src/tim.c:121-123`：PSC=7199、ARR=9，
T=(7199+1)*(9+1)/72MHz=1 ms。`PowerManager.cpp:1014-1018` 的 TIM2 IRQ
调用 handleErrorState；563-599 行故障时反复置 restartCoolDown=500，无故障递减，
减为零后的下一次调用允许输出，名义约500 ms（存在一个调度边界），并非当前Task周期。
本工程按当前25 kHz控制周期独立换算500 ms对应的12500次，并采用严格大于判定，不移植参考工程时钟。

旧工程 `Src/main.c:157-175`：16 MHz /4 *85 /2=170 MHz，APB1 /1；
`Src/tim.c:113-115` 的 TIM6 PSC=169、ARR=1999，周期2 ms。
`Src/stm32g4xx_it.c:360-365` 调用 power_ctrl_task；其中 power_error_check_advanced
在 app/power_ctrl_task.c:42 被注释，故旧保护检测不能称为实际启用。
若恢复该调用，381-429 行计数及大于10/1的判断单位是调用次数，首次置位在第11/2次，
分别名义22/4 ms，而不是10/1 ms。TIM7 的电流计算周期是20 us，也不是保护检测周期。
旧版重启计数由同一2 ms任务推动，且故障时不清零，不能照搬其500次为500 ms。

此前已按用户指示删除 `tools/test_adc_feedback.py`；本轮未删除文件，旧测试命令不再适用。
本轮仅做静态与语法检查，不执行专项运行测试、固件构建或上板验证；优化算法后续启用时的行为、临界区及完整控制路径耗时待测。

## 经典 CAN 通信

Procotol/bsp_can 适配 Taurus 的 Fdcanx_Init/Fdcanx_Transmit，仅使用 FDCAN1。
双ID过滤单个0x010，拒绝不匹配、扩展和远程帧；接收经典标准数据帧DLC为1～8，
短帧补零后交回调。发送固定8字节，忙或失败只计数丢弃，不等待和重试。
初始化回调先注册再启用通知和启动；RX单次最多读硬件FIFO深度3帧。

| ID | 数据 |
| --- | --- |
| TX 0x100 | bytes0～3电容电压V、bytes4～7底盘电流A，小端IEEE754 binary32 |
| TX 0x101 | byte0旧故障位，byte1旧状态，bytes2～7固定零 |
| RX 0x010 | byte0仅记录旧协议原始值，不参与超电开闭环运行模式控制 |

旧故障位仅映射bit0电容过压、bit2底盘过压、bit3电池欠压、bit7裁判故障。
本轮只发送CTRL/NORMAL=0、PROTECT=1，等待恢复也发送PROTECT，不再发送旧RESTART=2。
线上状态与内部枚举编号不同，不能直接复制内部状态。短路等其他内部故障只通过PROTECT反映，
旧故障字节不扩展，无法表达完整错误；不新增CAN失联保护。
Master在保存/恢复PRIMASK的短临界区复制电压、电流、错误和状态，编码和HAL发送在临界区外。
底层发送取自Taurus，线上格式取自用户指定旧SuperCap2026_7_17；不导入旧控制阈值或FD路径。

## 调度配置与验证边界

Master底层计数250 kHz、周期4 μs，重复计数249，通过MREP产生1 kHz普通保护/CAN中断。
Timer A/B的NVIC关闭；当前生成配置仍保留Timer B外设REP请求，不产生CPU业务回调，代理不改配置。
NVIC：ADC1 DMA=0，ADC2 DMA和Master=1，USART/UART DMA/FDCAN=3，SysTick=15。
周期、请求、预装载和优先级由CubeMX配置和静态审查保证，PowerCtrlTask_Init不重复核对。
UART固定为USART1/DMA1_Channel3普通接收，不在初始化中复查绑定和模式，UART模块为必选。
CAN仅在BSP配置过滤器时保留容量边界检查。g_supercap_can记录底盘模式及发送忙/失败次数。
采样、数值、500 ms恢复、CAN帧和最坏40 μs执行时间均待上板验证。
接受活动数组近似平均；DMA停更可能保持最后输出，超期可能合并事件并改变控制间隔，均无独立关断机制。
