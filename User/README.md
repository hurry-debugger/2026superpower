# 自维护固件目录

当前已有整数平均、浮点低通和 ADC 物理量换算模块，尚未接入 Keil 构建或中断调度。
新增代码前必须先阅读根目录 `AGENTS.md` 与 `docs/coding_style.md`，在对话中提交逐文件修改清单，
获得用户同意后再创建目录、代码或修改构建配置。

## 模块与规划

```text
User/
├── App/          # 应用状态、任务编排、保护策略与业务协作
├── Bsp/          # 已有 adc_convert.c/.h：板级通道及标称换算
├── Algorithm/    # 已有 filter.c/.h：整数平均、浮点低通
└── Config/       # 有明确单位、来源和适用范围的配置
```

App、Config 尚为规划；按实际模块需要并获批后再创建，不预先堆放空实现。
C/H 在模块目录中配对存放，文件名小写下划线。新模块可以从 `templates/module_template.c/.h`
提炼结构，但其中的状态接口仅供明确需要状态返回的场景；简单计算直接返回数值或使用 void。

App 可依赖 Bsp、Algorithm 和 Config；Bsp 不反向包含 App；Algorithm 不依赖 HAL。
更复杂的协议或控制拆分在后续方案中明确，不把旧工程目录原样复制进来。

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
#include "adc_convert.h"

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
    AdcConvert_Convert(adc_raw, adc_convert);
}
```

`AdcConvert_Convert()` 返回传入的 `adc_convert` 地址，无内部静态输出数组。
平均支持其他非零次数与步长；采用 uint32_t 累加、四舍五入到整数计数。
换算比例、偏置及硬件基础参数集中在 `Bsp/adc_convert.h`，当前为未标定的标称值。

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
 * @note 与上例共用 filter.h 和 adc_convert.h，固定反馈周期调用。
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
后续接入时需要将两个 `.c` 加入构建，并配置 `User/Algorithm`、`User/Bsp` 头文件路径；
本轮没有修改 Keil 配置、DMA、HRTIM 或回调。

## 与生成工程的边界

**禁止直接编辑 `.ioc`；CubeMX 生成代码仅允许修改获批的 `USER CODE BEGIN/END` 区域内部，
区域外内容及标记行保持原样。** 自维护业务模块优先放在 `User/`。
放在 User 中的生成文件仍受同样保护，不能通过移动代码规避边界。
启动和回调可在对应的 USER CODE 区域内接入，仍须先列出具体修改清单并获批。
本目录说明本身不授权添加调用或改写 Keil 工程；构建接入另行批准。
