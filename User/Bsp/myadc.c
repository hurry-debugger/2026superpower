/**
 * @file myadc.c
 * @brief 根据通道枚举选择标称参数，统一换算电压和电流。
 */

#include "myadc.h"

/* ==================== 公共接口实现 ==================== */

float Myadc_ConvertValue(uint16_t raw_count, float ratio, float offset_count)
{
    return ((float)raw_count - offset_count) * ratio;
}

float *Myadc_Convert(const uint16_t adc_raw[ADC_CHANNEL_COUNT],
                     float adc_convert[ADC_CHANNEL_COUNT])
{
    for(uint32_t channel = 0U; channel < ADC_CHANNEL_COUNT; channel++)
    {
        float ratio;
        float offset_count;

        // clang-format off
        switch((AdcChannel_e)channel)
        {
            case V_CHASSIS:
            {
                ratio = ADC_V_CHASSIS_RATIO;
                offset_count = ADC_V_CHASSIS_OFFSET_COUNT;
            }    break;

            case V_BAT:
            {
                ratio = ADC_V_BAT_RATIO;
                offset_count = ADC_V_BAT_OFFSET_COUNT;
            }    break;

            case I_BAT:
            {
                ratio = ADC_I_BAT_RATIO;
                offset_count = ADC_I_BAT_OFFSET_COUNT;
            }    break;

            case I_CHASSIS:
            {
                ratio = ADC_I_CHASSIS_RATIO;
                offset_count = ADC_I_CHASSIS_OFFSET_COUNT;
            }    break;

            case V_CAP:
            {
                ratio = ADC_V_CAP_RATIO;
                offset_count = ADC_V_CAP_OFFSET_COUNT;
            }    break;

            default:
            {
                /* channel 由循环界定，当前枚举下不会进入此分支。 */
                continue;
            }
        }
        // clang-format on

        adc_convert[channel] = Myadc_ConvertValue(adc_raw[channel], ratio, offset_count);
    }

    return adc_convert;
}
