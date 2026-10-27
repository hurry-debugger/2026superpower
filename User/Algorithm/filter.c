/**
 * @file filter.c
 * @brief 带采样步长的整数平均实现。
 */

#include "filter.h"

/* ==================== 公共接口实现 ==================== */

uint16_t Filter_AverageU16(const uint16_t *samples, uint16_t sample_count, uint16_t sample_stride)
{
    uint32_t sum = 0U;

    for(uint32_t i = 0U; i < sample_count; i++)
    {
        sum += samples[i * sample_stride];
    }

    /* 最大和为 65535 * 65535，加上舍入量仍在 uint32_t 范围内。 */
    return (uint16_t)((sum + sample_count / 2U) / sample_count);
}
