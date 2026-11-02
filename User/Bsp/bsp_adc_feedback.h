/** @file bsp_adc_feedback.h
 * @brief 校准换算后的五路采样发布，不访问 DMA、不进行平均或拟合。
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

/**
 * @brief 发布整组物理量并检查有限性。
 * @param[in] adc_convert 非空稳定数组，ADC_CHANNEL_COUNT 个 V/A 元素。
 * @param[in] valid 采样块完成、无覆盖等问题时为 true。
 * @param[in] now_ms 与 HAL_GetTick() 同源的毫秒时刻。
 * @note 唯一串行调用者先发布再控制；禁止与读取并发。
 */
void BspAdcFeedback_Publish(const float adc_convert[ADC_CHANNEL_COUNT], bool valid, uint32_t now_ms);
/** @brief 标记无效，保留最近数据供观察；调用上下文同 Publish。 */
void BspAdcFeedback_Invalidate(void);

#endif /* SUPERPOWER_BSP_ADC_FEEDBACK_H */
