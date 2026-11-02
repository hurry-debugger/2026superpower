/** @file bsp_adc_feedback.h
 * @brief 双 ADC 完整 DMA 块交接、十次整数平均与五路反馈发布。
 */
#ifndef SUPERPOWER_BSP_ADC_FEEDBACK_H
#define SUPERPOWER_BSP_ADC_FEEDBACK_H

#include "adccalibration.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float adc_convert[ADC_CHANNEL_COUNT]; /**< AdcChannel_e 顺序，电压 V、电流 A。 */
    bool valid; /**< 完整采样且数值有限，不代替校准检查。 */
    uint32_t update_ms; /**< 最近一次发布的单调毫秒时刻。 */
    uint32_t sequence; /**< 每次发布递增，可自然回绕。 */
} BspAdcFeedback_t;

/** @brief 所属 .c 唯一写入，其他模块按约定只读。 */
extern BspAdcFeedback_t g_bsp_adc_feedback;

/** @brief 每通道每块十次采样；250 kHz 触发对应 40 us，来源为本轮调度约定。 */
#define BSP_ADC_FEEDBACK_SAMPLE_COUNT (10U)

/**
 * @brief 共同初始化两路采集状态并启动 ADC1[80]/ADC2[20] 循环 DMA。
 * @pre UID 匹配及两个 ADC 硬件校准已完成；HRTIM 未启动，功率输出关闭。
 * @note 仅启动阶段调用；采集故障锁存后需停止计数器和输出，再共同初始化才可恢复。
 *       HAL 失败进入 Error_Handler；不修改 CubeMX 配置或重新拟合。
 */
void BspAdcFeedback_Init(void);
/**
 * @brief Master 中处理最近同序号的两路稳定块，平均、换算后发布一次。
 * @param[in] now_ms 与 HAL_GetTick 同源的处理时刻，单位 ms。
 * @pre Master 优先级高于两个 ADC DMA IRQ；DMA 回调只复制块和发布元数据。
 * @note 无新配对块时不重复发布；采集故障时标记反馈无效，不自动重新同步。
 */
void BspAdcFeedback_Process(uint32_t now_ms);

/**
 * @brief 发布整组物理量并检查有限性。
 * @param[in] adc_convert 非空稳定数组，ADC_CHANNEL_COUNT 个 V/A 元素。
 * @param[in] valid 采样块完成、无覆盖等问题时为 true。
 * @param[in] now_ms 与 HAL_GetTick() 同源的毫秒时刻。
 * @note 运行期仅 Master 的 Process 写入，随后同上下文控制；其他读取者需取得一致快照。
 */
void BspAdcFeedback_Publish(const float adc_convert[ADC_CHANNEL_COUNT], bool valid, uint32_t now_ms);
/** @brief 标记无效，保留最近数据供观察；调用上下文同 Publish。 */
void BspAdcFeedback_Invalidate(void);

#endif /* SUPERPOWER_BSP_ADC_FEEDBACK_H */
