/** @file bsp_adc_feedback.c
 * @brief 五路物理量的单写入者发布实现。
 */
#include "bsp_adc_feedback.h"

#include <math.h>

BspAdcFeedback_t g_bsp_adc_feedback;

void BspAdcFeedback_Publish(const float adc_convert[ADC_CHANNEL_COUNT], bool valid, uint32_t now_ms)
{
    g_bsp_adc_feedback.valid = false;
    for(uint32_t channel = 0U; channel < ADC_CHANNEL_COUNT; channel++)
    {
        g_bsp_adc_feedback.adc_convert[channel] = adc_convert[channel];
        if(!isfinite(adc_convert[channel]))
            valid = false;
    }
    g_bsp_adc_feedback.update_ms = now_ms;
    g_bsp_adc_feedback.sequence++;
    g_bsp_adc_feedback.valid = valid;
}

void BspAdcFeedback_Invalidate(void)
{
    g_bsp_adc_feedback.valid = false;
}
