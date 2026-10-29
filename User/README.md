# 自维护固件目录

当前已有整数平均、浮点低通，以及 UID 匹配、手填校准记录、ADC 统一换算和 HRTIM 输出模块，
均已登记 Keil；HRTIM 输出模块尚未接入运行。
启动时已接入 UID 匹配、拟合及功率输出启动门槛；平均、换算及低通的周期调度尚未接入。
新增代码前必须先阅读根目录 `AGENTS.md` 与 `docs/coding_style.md`，在对话中提交逐文件修改清单，
获得用户同意后再创建目录、代码或修改构建配置。

## 模块与规划

```text
User/
├── App/          # 应用状态、任务编排、保护策略与业务协作
├── Bsp/          # UID、adc_calibration_record、adccalibration、bsp_hrtim 的 C/H
├── Algorithm/    # 已有 filter.c/.h：整数平均、浮点低通
└── Config/       # 有明确单位、来源和适用范围的配置
```

App、Config 尚为规划；按实际模块需要并获批后再创建，不预先堆放空实现。
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
硬件理论参数在 `Bsp/adccalibration.h`。统一公式为 `raw_count * gain + bias`，
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
`User/Algorithm`、`User/Bsp` 包含目录；未执行固件构建。所有函数实现在 `.c`，头文件
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
