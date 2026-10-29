/**
 * @file adccalibration.c
 * @brief 启动时拟合五路采集记录，统一使用缓存增益和偏置进行物理量换算。
 * @note 拟合失败的通道仅保留理论值供观察；是否启动控制由全局状态门槛决定。
 */

#include "adccalibration.h"

#include <float.h>
#include <math.h>

/* ==================== 私有数据 ==================== */

/** @brief 图纸标称系数，偏置单位 V/A，由计数零点乘增益并取负得到。 */
static const AdcCalibrationCoeff_t s_theory_coefficients[ADC_CHANNEL_COUNT] =
{
    [V_CHASSIS] = {ADC_V_CHASSIS_RATIO, -ADC_V_CHASSIS_OFFSET_COUNT * ADC_V_CHASSIS_RATIO},
    [V_BAT] = {ADC_V_BAT_RATIO, -ADC_V_BAT_OFFSET_COUNT * ADC_V_BAT_RATIO},
    [I_BAT] = {ADC_I_BAT_RATIO, -ADC_I_BAT_OFFSET_COUNT * ADC_I_BAT_RATIO},
    [I_CHASSIS] = {ADC_I_CHASSIS_RATIO, -ADC_I_CHASSIS_OFFSET_COUNT * ADC_I_CHASSIS_RATIO},
    [V_CAP] = {ADC_V_CAP_RATIO, -ADC_V_CAP_OFFSET_COUNT * ADC_V_CAP_RATIO}
};

/** @brief 启动时更新，消费者运行后只读；未调用初始化也可用理论值观察。 */
static AdcCalibrationCoeff_t s_coefficients[ADC_CHANNEL_COUNT] =
{
    [V_CHASSIS] = {ADC_V_CHASSIS_RATIO, -ADC_V_CHASSIS_OFFSET_COUNT * ADC_V_CHASSIS_RATIO},
    [V_BAT] = {ADC_V_BAT_RATIO, -ADC_V_BAT_OFFSET_COUNT * ADC_V_BAT_RATIO},
    [I_BAT] = {ADC_I_BAT_RATIO, -ADC_I_BAT_OFFSET_COUNT * ADC_I_BAT_RATIO},
    [I_CHASSIS] = {ADC_I_CHASSIS_RATIO, -ADC_I_CHASSIS_OFFSET_COUNT * ADC_I_CHASSIS_RATIO},
    [V_CAP] = {ADC_V_CAP_RATIO, -ADC_V_CAP_OFFSET_COUNT * ADC_V_CAP_RATIO}
};

/* ==================== 私有接口声明 ==================== */

/**
 * @brief 校验全部固定采集点，以中心化最小二乘拟合单通道。
 * @param[in] record 稳定的只读记录。
 * @param[in] channel 有效五路通道之一。
 * @param[out] coefficient 仅成功时写入 gain 与 bias，单位为对应 V/A。
 * @return AdcCalibrationChannelError_e 问题位，零表示成功。
 * @pre 指针非空；仅在启动初始化调用，不在高速反馈中拟合。
 */
static uint32_t AdcCalibration_FitChannel(const AdcCalibrationRecord_t *record,
                                         AdcChannel_e channel,
                                         AdcCalibrationCoeff_t *coefficient);

/* ==================== 公共接口实现 ==================== */

AdcCalibrationState_t AdcCalibration_Init(SupercapId_e supercap_id)
{
    AdcCalibrationState_t state =
    {
        .supercap_id = SUPERCAP_ID_UNKNOWN
    };

    for(uint32_t channel = 0U; channel < ADC_CHANNEL_COUNT; channel++)
    {
        s_coefficients[channel] = s_theory_coefficients[channel];
    }

    if(supercap_id == SUPERCAP_ID_UNKNOWN)
    {
        state.board_errors = ADC_CAL_BOARD_ERR_NO_RECORD;
        return state;
    }

    if((int)supercap_id < 0 || supercap_id >= SUPERCAP_COUNT)
    {
        state.board_errors = ADC_CAL_BOARD_ERR_INVALID_ID;
        return state;
    }

    state.supercap_id = supercap_id;

    for(uint32_t channel = 0U; channel < ADC_CHANNEL_COUNT; channel++)
    {
        AdcCalibrationCoeff_t coefficient;
        uint32_t errors = AdcCalibration_FitChannel(
            &g_adc_calibration_records[supercap_id], (AdcChannel_e)channel, &coefficient);

        state.channel_errors[channel] = errors;

        if(errors == ADC_CAL_CHANNEL_ERR_NONE)
        {
            s_coefficients[channel] = coefficient;
            state.valid_channel_mask |= 1U << channel;
        }
    }

    return state;
}

float AdcCalibration_ConvertValue(uint16_t raw_count, float gain, float bias)
{
    return (float)raw_count * gain + bias;
}

float *AdcCalibration_Convert(const uint16_t adc_raw[ADC_CHANNEL_COUNT],
                              float adc_convert[ADC_CHANNEL_COUNT])
{
    for(uint32_t channel = 0U; channel < ADC_CHANNEL_COUNT; channel++)
    {
        adc_convert[channel] = AdcCalibration_ConvertValue(
            adc_raw[channel], s_coefficients[channel].gain, s_coefficients[channel].bias);
    }

    return adc_convert;
}

/* ==================== 私有接口实现 ==================== */

static uint32_t AdcCalibration_FitChannel(const AdcCalibrationRecord_t *record,
                                         AdcChannel_e channel,
                                         AdcCalibrationCoeff_t *coefficient)
{
    const float *raw = record->calibration_data[ADC_CAL_RAW_ROW(channel)];
    const float *real = record->calibration_data[ADC_CAL_REAL_ROW(channel)];
    uint32_t errors = ADC_CAL_CHANNEL_ERR_NONE;
    double raw_sum = 0.0;
    double real_sum = 0.0;

    for(uint32_t point = 0U; point < ADC_CAL_POINT_COUNT; point++)
    {
        if(isnan(raw[point]) || isnan(real[point]))
        {
            errors |= ADC_CAL_CHANNEL_ERR_MISSING_POINT;
        }

        if((!isfinite(raw[point]) && !isnan(raw[point]))
           || (!isfinite(real[point]) && !isnan(real[point])))
        {
            errors |= ADC_CAL_CHANNEL_ERR_NONFINITE;
        }

        if(isfinite(raw[point])
           && (raw[point] < 0.0f || raw[point] > (float)ADC_FULL_SCALE_COUNT))
        {
            errors |= ADC_CAL_CHANNEL_ERR_COUNT_RANGE;
        }

        if(isfinite(raw[point]) && isfinite(real[point]))
        {
            raw_sum += (double)raw[point];
            real_sum += (double)real[point];
        }
    }

    /* 不丢弃坏点继续拟合；先收齐当前通道可判断的数据问题。 */
    if(errors != ADC_CAL_CHANNEL_ERR_NONE)
    {
        return errors;
    }

    double raw_mean = raw_sum / (double)ADC_CAL_POINT_COUNT;
    double real_mean = real_sum / (double)ADC_CAL_POINT_COUNT;
    double raw_variance = 0.0;
    double covariance = 0.0;

    for(uint32_t point = 0U; point < ADC_CAL_POINT_COUNT; point++)
    {
        double raw_delta = (double)raw[point] - raw_mean;
        double real_delta = (double)real[point] - real_mean;

        raw_variance += raw_delta * raw_delta;
        covariance += raw_delta * real_delta;
    }

    if(raw_variance == 0.0)
    {
        return ADC_CAL_CHANNEL_ERR_ZERO_VARIANCE;
    }

    double gain = covariance / raw_variance;
    double bias = real_mean - gain * raw_mean;

    if(!isfinite(gain) || !isfinite(bias))
    {
        return ADC_CAL_CHANNEL_ERR_NUMERIC_RANGE;
    }

    if(gain <= 0.0)
    {
        errors |= ADC_CAL_CHANNEL_ERR_GAIN_NONPOSITIVE;
    }

    if(fabs(gain) > (double)FLT_MAX || fabs(bias) > (double)FLT_MAX)
    {
        errors |= ADC_CAL_CHANNEL_ERR_NUMERIC_RANGE;
    }

    if(errors != ADC_CAL_CHANNEL_ERR_NONE)
    {
        return errors;
    }

    AdcCalibrationCoeff_t fitted = {(float)gain, (float)bias};

    if(!isfinite(fitted.gain) || fitted.gain <= 0.0f || !isfinite(fitted.bias))
    {
        return ADC_CAL_CHANNEL_ERR_NUMERIC_RANGE;
    }

    /* 强制分别落到 float，防止乘法溢出被后续负偏置抵消而漏检。 */
    volatile float full_scale_product = (float)ADC_FULL_SCALE_COUNT * fitted.gain;
    volatile float full_scale_value = full_scale_product + fitted.bias;
    volatile float zero_value = 0.0f * fitted.gain + fitted.bias;

    if(!isfinite(full_scale_product) || !isfinite(full_scale_value) || !isfinite(zero_value))
    {
        return ADC_CAL_CHANNEL_ERR_NUMERIC_RANGE;
    }

    *coefficient = fitted;
    return ADC_CAL_CHANNEL_ERR_NONE;
}
