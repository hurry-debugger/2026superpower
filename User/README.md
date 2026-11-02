# 自维护固件目录

当前已有滤波、UID/校准、HRTIM、BSP 数据发布及四环控制模块，均已登记 Keil。
采样发布、裁判发布及控制任务尚未接入运行；不得把模块登记当成已启动保护。
启动时已接入 UID 匹配、拟合及功率输出启动门槛；平均、换算及低通的周期调度尚未接入。
新增代码前必须先阅读根目录 `AGENTS.md` 与 `docs/coding_style.md`，在对话中提交逐文件修改清单，
获得用户同意后再创建目录、代码或修改构建配置。

## 模块与规划

```text
User/
├── App/          # 应用状态、任务编排、保护策略与业务协作
├── Bsp/          # UID、记录、HRTIM、采样及裁判数据发布
├── Calibration/  # adccalibration.c/.h，校准算法及理论换算
├── Algorithm/    # 已有 filter.c/.h：整数平均、浮点低通
└── Config/       # 有明确单位、来源和适用范围的配置
```

App、Config、Calibration 已登记构建；Calibration 迁移保留原有 AdcCalibration 接口。
C/H 在模块目录中配对存放，文件名小写下划线；UID.c/.h 为用户指定命名例外。
新模块可以从 `templates/module_template.c/.h`
提炼结构，但其中的状态接口仅供明确需要状态返回的场景；简单计算直接返回数值或使用 void。

App 可依赖 Bsp、Algorithm 和 Config；Bsp 不反向包含 App；Algorithm 不依赖 HAL。
更复杂的协议或控制拆分在后续方案中明确，不把旧工程目录原样复制进来。

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
全局状态记录当前 UID、超电 ID、板卡问题、每通道问题位及有效通道掩码。NAN 缺失、其他
非有限值、计数越界、零方差、非正增益、数值溢出分别报告，不静默丢弃坏点继续拟合。
负 Bias 合法。只有 `board_errors == 0U` 且有效掩码为 `ADC_CAL_ALL_CHANNELS_MASK` 才允许
main 启动功率输出；无匹配记录和任一通道失败均禁止控制。

main 先禁用输出，再匹配拟合；保留 ADC 硬件自校准、DMA 和采样触发计数器启动。
这是启动门槛，不包含运行期测量故障关断。再次匹配或拟合只能在功率输出关闭、消费者尚未
运行时进行；不能与中断换算并发。初始化后状态和系数保持只读，不默认使用 volatile。

## 平均后转换

调用方持有 `uint16_t adc_raw[ADC_CHANNEL_COUNT]` 和
`float adc_convert[ADC_CHANNEL_COUNT]`，按 `adc_raw[V_CHASSIS]`、
`adc_convert[I_CHASSIS]` 访问。电压单位为 V，电流单位为 A。
`adc_raw` 只表示平均后的计数；多次采样缓冲区另行命名。

以下为应用层接入示例，尚未添加到固件。输入必须是稳定的完整采样块：
ADC1 含 10 次四通道扫描，共 40 个半字；ADC2 含 10 次单通道采样，共 10 个半字。
当前 `main.c` 的 4/1 元素缓冲区不能直接作为本示例输入。
调用方负责两路块的对应关系和处理期间不被 DMA 覆盖；不可通过去掉 volatile 强行满足契约。

```c
#include "filter.h"
#include "adccalibration.h"

#define FEEDBACK_SAMPLE_COUNT (10U)
#define FEEDBACK_ADC1_STRIDE (4U)
#define FEEDBACK_ADC2_STRIDE (1U)

/**
 * @brief 对完整采样块先做整数平均，再输出五路物理量。
 * @pre 两个输入块已完成采集且调用期间稳定；输出各有 ADC_CHANNEL_COUNT 个元素。
 * @note 前四个通道枚举顺序与当前 ADC1 Rank 顺序一致，ADC2 单独对应 V_CAP。
 */
void Feedback_ConvertBlock(const uint16_t *adc1_samples, const uint16_t *adc2_samples,
                          uint16_t adc_raw[ADC_CHANNEL_COUNT],
                          float adc_convert[ADC_CHANNEL_COUNT])
{
    for(uint32_t channel = 0U; channel < FEEDBACK_ADC1_STRIDE; channel++)
    {
        adc_raw[channel] = Filter_AverageU16(&adc1_samples[channel], FEEDBACK_SAMPLE_COUNT,
                                           FEEDBACK_ADC1_STRIDE);
    }
    adc_raw[V_CAP] = Filter_AverageU16(adc2_samples, FEEDBACK_SAMPLE_COUNT,
                                     FEEDBACK_ADC2_STRIDE);
    AdcCalibration_Convert(adc_raw, adc_convert);
}
```

`AdcCalibration_Convert()` 返回传入的 `adc_convert` 地址，无内部静态输出数组。
返回地址不代表校准成功；无有效校准时结果仅供观察，不能作为允许控制的依据。
平均支持其他非零次数与步长；采用 uint32_t 累加、四舍五入到整数计数。
硬件理论参数在 `Calibration/adccalibration.h`。统一公式为 `raw_count * gain + bias`，
理论偏置为 `-offset_count * ratio`，拟合偏置直接来自实测数据，不再扣一次零点。
`AdcCalibration_ConvertValue()` 的第三参数单位为 V/A，与旧计数零点接口不同。

## 可选浮点低通

默认直接使用 `adc_convert` 作为反馈。需要低通时，在换算后执行下列示例，
由调用方长期保存 `adc_lowpass`，以其作为滤波后的反馈。不同通道不共用历史值。
首次传入 `first_block = true`，之后传入 false；重新开始采集或丢失历史时重新初始化。
`alpha` 由调用方按实际反馈周期选定，范围为 [0, 1]；这里不指定截止频率或默认系数。

```c
#include <stdbool.h>

/**
 * @brief 更新调用方保存的各通道物理量历史，首次直接使用当前换算值。
 * @pre 两个数组均有 ADC_CHANNEL_COUNT 个元素，alpha 有效；非首次时历史已初始化。
 * @note 与上例共用 filter.h 和 adccalibration.h，固定反馈周期调用。
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
filter 及三个 ADC 相关 Bsp 模块的 C/H 已登记 `MDK-ARM/2026superpower.uvprojx`，沿用现有
`User/Algorithm`、`User/Bsp`、`User/Calibration` 包含目录；未执行固件构建。所有函数实现在 `.c`，头文件
只保留声明、类型和宏。没有新增 DMA、平均调度或反馈回调。

## HRTIM 广义占空比底层模块

`Bsp/bsp_hrtim.c/.h` 已提供四个 `void` 接口并登记 Keil；尚未在 main 中调用。
当前 HRTIM 预装载和复位更新仍关闭，不能直接调用占空比更新接口运行本板。

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

### 后续 CubeMX 配置前提

以下是后续单独审批和操作 CubeMX 时的要求，本轮没有执行或重新生成：

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

后续接入顺序为：完成并核对 CubeMX 配置 → MX_HRTIM1_Init() → BspHrtim_Init() →
准备采样并依据实际反馈确定启动 g → SetGeneralizedDuty(g) → 启动计数器并至少运行一个
完整 PWM 周期 → 确认两侧装载、完整 ADC 校准及其他启动条件 → EnableOutput()。
计数器由调用方管理；已运行时也须从最近一次设置后等待一个完整周期再开启输出。
应用层周期调用设置函数，停机使用 DisableOutput()。本段是接入契约，不是已运行的代码。

## 与生成工程的边界

**禁止直接编辑 `.ioc`；CubeMX 生成代码仅允许修改获批的 `USER CODE BEGIN/END` 区域内部，
区域外内容及标记行保持原样。** 自维护业务模块优先放在 `User/`。
放在 User 中的生成文件仍受同样保护，不能通过移动代码规避边界。
启动和回调可在对应的 USER CODE 区域内接入，仍须先列出具体修改清单并获批。
后续新增或重命名 C/H 必须把 Keil 分组与头文件路径维护纳入同一清单；
本目录说明不单独授权写文件或添加运行时调用。

## 应用层控制与后续接入

`App/power_ctrl.c/.h` 提供 `PowerCtrl_Init(void)`、`PowerCtrl_Task(void)`，
主要实现依次为 Init、Task、ModeSwitch、LoopCalcu。Init 内按 Taurus 形式逐环调用
PID_Init，先清零实例；原 PID 计算代码保持，死区为 0，不启用电机堵转检测。
误差恰为零仍输出零、增量式 Ki=0 清零的原行为保留。

### 编译模式与配置

在 `Core/Inc/main.h` 的 USER CODE Private defines 中用注释三选一：

| 宏 | 行为 |
| --- | --- |
| POWER_CTRL_MATCH | 有效裁判数据进入 45 J 能量环；裁判超过 300 ms 未更新则保护 |
| POWER_CTRL_NO_REFEREE | 默认选择，固定功率初值 80 W，自动启动；允许接口设为负功率 |
| POWER_CTRL_OPEN_LOOP | 手动 g，不计算 PID；本地使能初始关闭，电容达到 29 V 禁止输出 |

`Config/power_ctrl_config.h` 集中保存参数。PARAMETERS_READY、PWM_READY 默认均为 0，
Task、采样发布、PWM 的周期（us）以及全部 PID 系数也是待填模板；填写及接入完成后
再开启对应就绪标志。Task 必须唯一、周期固定且不短于完整 PWM 周期。
所有闭环每个 Task 计算一次；系数按调用次数定义，不在库中自动乘除 dt。

### 数据与调用上下文

`BspAdcFeedback_Publish(adc_convert, valid, now_ms)` 发布五路稳定物理量，
`BspRefereeData_Publish(power_limit_w, buffer_energy_j, now_ms)` 发布有效裁判值。
数据分别唯一定义在所属 BSP .c；其他模块 extern 读取，按约定禁止直接赋值。
更新与 Task 必须在同一串行上下文执行，不允许中断异步写入；volatile 不提供一致性。
Publish/Invalidate 不访问 HAL，不接管 DMA、平均、低通、UART 或协议校验。

将来运行调用顺序：完整校准 → Bsp 数据发布 → PowerCtrl_Init（硬件初始化完成后）→
周期内先发布采样/已解析裁判，再 PowerCtrl_Task。单调毫秒时基使用 HAL_GetTick，
发布参数 now_ms 必须与它同源。每次 Task 按序号判断是否更新，序号未变时累计配置的
控制周期 us；超过两倍发布周期，在第一个超时的控制调用中检出。毫秒时间戳另查长时间
陈旧数据。不能在丢失调度后用虚假的固定周期继续调用；实时调度与执行预算待验证。

本轮 main 没有接入这些调用，仍保留原来的直接输出启动代码；接入控制时必须一起移除
其他输出写入者、完成预装载配置，不能只添加 Task。当前校准记录仍为待填模板。

### 控制关系

- 能量环 P_raw=P_limit-PID_energy；全局放电下限 -450 W，再由当前 15*Vcap W
  收缩上下限，上限同时不超过 P_limit。PID 修正限值允许大于 P_limit，使负目标可达。
- 功率反馈是 V_CHASSIS*I_CHASSIS，正值充电、负值放电；功率位置环输出端口目标电流。
- 目标电流对称限幅 ±15*Vcap/Vchassis；不设置 5 V 禁充禁放或 4 A 固定限制。
- 电容超过 26.1 V 时电压环以 29 V 为目标，输出非负充电上限；达到 29 V 禁止正目标，
  仍可放电。30 V 是独立硬故障阈值。计量和功率模型不能代替电容实测过流保护。
- g=Vcap/Vchassis+电流位置 PID 修正。最终饱和及电压竞争阻止同向积分，内环饱和
  向功率/能量外环传播；App 包装预查计算溢出，PID 库计算本身不修改。
- App 只调用 BspHrtim 接口；逻辑 A/chassis 对应本板 Timer B，电容对应 Timer A。
  电机回馈升高底盘电压时限流随当前 Vchassis 更新，另有底盘过压保护；不是母线稳压环。

### 三状态与故障时间

上电 PROTECT_MODE；条件有效后 RESTART_MODE 等待连续正常 500 ms；保持输出关闭并
设置实际电压比，然后再等待 ceil(PWM周期us/1000)+1 个毫秒 tick，覆盖写入时刻的相位。
最后进入 CTRL_MODE。等待或装载准备期间失效立即回到 PROTECT，清零等待及环路历史。
已有连续故障位不会因为无效采样被误清除。故障检测与关断只能在 Task 上下文运行，
不是异步硬件紧急保护；保护响应延迟还包括采样、调度、计算及 HAL 执行时间。

| 条件 | 触发 | 恢复 |
| --- | --- | --- |
| Vcap>30 V | 连续 10 次 Task | Vcap<=30 V |
| Vchassis>43 V | 连续 1 次 Task | Vchassis<33 V |
| Vbat<10 V | 连续 10 次 Task | Vbat>=10 V |
| 裁判超时 | 上场无有效数据或超过 300 ms | 新的有效数据 |
| 校准/采样/数值/配置异常 | 本次检查不满足 | 原因消除并重新等待 |

上述 10/1/10 是用户批准的控制周期数，不是毫秒。计数条件成立时饱和递增，
不成立时清零；已置位按恢复条件解除。不会继续使用旧版 CAP_MIN_V=7.5 V 故障。

### 参考工程计时核查（静态配置，不是实板测量）

港科 RM2024 `Core/Src/main.c:133-150`：8 MHz HSE /1 *9=72 MHz，APB1 /2，
定时器时钟乘2回到72 MHz；`Core/Src/tim.c:121-123`：PSC=7199、ARR=9，
T=(7199+1)*(9+1)/72MHz=1 ms。`PowerManager.cpp:1014-1018` 的 TIM2 IRQ
调用 handleErrorState；563-599 行故障时反复置 restartCoolDown=500，无故障递减，
减为零后的下一次调用允许输出，名义约500 ms（存在一个调度边界），并非当前Task周期。
本工程独立选用500 ms单调计时，不移植它的计数时钟。

旧工程 `Src/main.c:157-175`：16 MHz /4 *85 /2=170 MHz，APB1 /1；
`Src/tim.c:113-115` 的 TIM6 PSC=169、ARR=1999，周期2 ms。
`Src/stm32g4xx_it.c:360-365` 调用 power_ctrl_task；其中 power_error_check_advanced
在 app/power_ctrl_task.c:42 被注释，故旧保护检测不能称为实际启用。
若恢复该调用，381-429 行计数及大于10/1的判断单位是调用次数，首次置位在第11/2次，
分别名义22/4 ms，而不是10/1 ms。TIM7 的电流计算周期是20 us，也不是保护检测周期。
旧版重启计数由同一2 ms任务推动，且故障时不清零，不能照搬其500次为500 ms。

本轮按用户指示删除 `tools/test_adc_feedback.py`；旧测试说明中的对应命令已不适用。
未执行专项测试、固件构建或上板验证。PID 预查使用 double，其目标执行时间待测量。
