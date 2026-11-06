/**
 * @file adccalibration.c
 * @note 校准算法归属 User/Calibration；UID 与记录数据仍由 Bsp 提供。
 * @brief 启动时拟合五路采集记录，统一使用缓存增益和偏置进行物理量换算。
 * @note 拟合失败的通道仅保留理论值供观察；是否启动控制由全局状态门槛决定。
 */

#include "adccalibration.h"

#include "power_ctrl_config.h"

#include <float.h>
#include <math.h>

/* ==================== 私有数据 ==================== */

/** @brief 启动时更新，消费者运行后只读；未调用初始化也可用理论值观察。 */
static AdcCalibrationCoeff_t s_coefficients[ADC_CHANNEL_COUNT] =
{
    [V_CHASSIS] = {ADC_V_CHASSIS_RATIO, -ADC_V_CHASSIS_OFFSET_COUNT * ADC_V_CHASSIS_RATIO},
    [V_BAT] = {ADC_V_BAT_RATIO, -ADC_V_BAT_OFFSET_COUNT * ADC_V_BAT_RATIO},
    [I_BAT] = {ADC_I_BAT_RATIO, -ADC_I_BAT_OFFSET_COUNT * ADC_I_BAT_RATIO},
    [I_CHASSIS] = {ADC_I_CHASSIS_RATIO, -ADC_I_CHASSIS_OFFSET_COUNT * ADC_I_CHASSIS_RATIO},
    [V_CAP] = {ADC_V_CAP_RATIO, -ADC_V_CAP_OFFSET_COUNT * ADC_V_CAP_RATIO}
};

/** @brief 直接复用控制配置的物理量上限；电压单位 V，电流单位 A。 */
static const float s_value_limits[ADC_CHANNEL_COUNT] =
{
    [V_CHASSIS] = POWER_CTRL_CHASSIS_OVERVOLTAGE_V,
    [V_BAT] = POWER_CTRL_CHASSIS_OVERVOLTAGE_V,
    [I_BAT] = POWER_CTRL_CAP_CURRENT_MAX_A,
    [I_CHASSIS] = POWER_CTRL_CAP_CURRENT_MAX_A,
    [V_CAP] = POWER_CTRL_CAP_OVERVOLTAGE_V
};

/* ==================== 私有接口声明 ==================== */

/**
 * @brief 校验全部固定采集点，以中心化最小二乘拟合单通道。
 * @param[in] record 稳定的只读记录。
 * @param[in] channel 有效五路通道之一。
 * @param[out] coefficient 仅成功时写入 gain 与 bias，单位为对应 V/A。
 * @return 本通道错误结构体，所有错误成员为 0 表示成功。
 * @pre 指针非空；仅在启动初始化调用，不在高速反馈中拟合。
 */
static AdcCalibrationChannelError_t AdcCalibration_FitChannel(
    const AdcCalibrationRecord_t *record, AdcChannel_e channel,
    AdcCalibrationCoeff_t *coefficient);

/* ==================== 公共接口实现 ==================== */

AdcCalibrationState_t AdcCalibration_Init(SupercapId_e supercap_id)
{
    AdcCalibrationState_t state =
    {
        .supercap_id = SUPERCAP_ID_UNKNOWN
    };

    /* 每次初始化先恢复理论值，避免失败通道沿用上次校准系数。 */
    s_coefficients[V_CHASSIS].gain = ADC_V_CHASSIS_RATIO;
    s_coefficients[V_CHASSIS].bias =
        -ADC_V_CHASSIS_OFFSET_COUNT * ADC_V_CHASSIS_RATIO;
    s_coefficients[V_BAT].gain = ADC_V_BAT_RATIO;
    s_coefficients[V_BAT].bias =
        -ADC_V_BAT_OFFSET_COUNT * ADC_V_BAT_RATIO;
    s_coefficients[I_BAT].gain = ADC_I_BAT_RATIO;
    s_coefficients[I_BAT].bias =
        -ADC_I_BAT_OFFSET_COUNT * ADC_I_BAT_RATIO;
    s_coefficients[I_CHASSIS].gain = ADC_I_CHASSIS_RATIO;
    s_coefficients[I_CHASSIS].bias =
        -ADC_I_CHASSIS_OFFSET_COUNT * ADC_I_CHASSIS_RATIO;
    s_coefficients[V_CAP].gain = ADC_V_CAP_RATIO;
    s_coefficients[V_CAP].bias =
        -ADC_V_CAP_OFFSET_COUNT * ADC_V_CAP_RATIO;

    if(supercap_id == SUPERCAP_ID_UNKNOWN)
    {
        state.board_errors.no_record = 1U;
    }
    else if((int)supercap_id < 0 || supercap_id >= SUPERCAP_COUNT)
    {
        state.board_errors.invalid_id = 1U;
    }
    else
    {
        state.supercap_id = supercap_id;

        for(uint32_t channel = 0U; channel < ADC_CHANNEL_COUNT; channel++)
        {
            state.channel_errors[channel] = AdcCalibration_FitChannel(
                &g_adc_calibration_records[supercap_id], (AdcChannel_e)channel,
                &s_coefficients[channel]);

            if(!state.channel_errors[channel].missing_point
               && !state.channel_errors[channel].nonfinite
               && !state.channel_errors[channel].count_range
               && !state.channel_errors[channel].zero_variance
               && !state.channel_errors[channel].gain_nonpositive
               && !state.channel_errors[channel].numeric_range
               && !state.channel_errors[channel].value_range)
            {
                state.valid_channel_mask |= 1U << channel;
            }
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

static AdcCalibrationChannelError_t AdcCalibration_FitChannel(
    const AdcCalibrationRecord_t *record, AdcChannel_e channel,
    AdcCalibrationCoeff_t *coefficient)
{
    /* 返回本次拟合错误；失败前不能复用或覆盖调用方已有的换算系数。 */
    AdcCalibrationChannelError_t errors = {0};
    const float *raw = record->calibration_data[ADC_CAL_RAW_ROW(channel)];
    const float *real = record->calibration_data[ADC_CAL_REAL_ROW(channel)];
    const float min_value = (channel == I_BAT || channel == I_CHASSIS)
                                ? -s_value_limits[channel] : 0.0f;
    float raw_min = (float)ADC_FULL_SCALE_COUNT;
    float raw_max = 0.0f;
    double raw_sum = 0.0;
    double real_sum = 0.0;

    for(uint32_t point = 0U; point < ADC_CAL_POINT_COUNT; point++)
    {
        /* 必须落在闭区间内；NaN 和无穷也会被范围比较拒绝。 */
        if(!(raw[point] >= 0.0f && raw[point] <= (float)ADC_FULL_SCALE_COUNT))
        {
            errors.count_range = 1U;
        }
        else if(!(real[point] >= min_value && real[point] <= s_value_limits[channel]))
        {
            errors.value_range = 1U;
        }

        if(errors.count_range || errors.value_range)
            break;

        if(raw[point] < raw_min)
        {
            raw_min = raw[point];
        }

        if(raw[point] > raw_max)
        {
            raw_max = raw[point];
        }

        raw_sum += (double)raw[point];
        real_sum += (double)real[point];
    }

    if(!errors.count_range && !errors.value_range)
    {
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
            errors.zero_variance = 1U;
        }
        else
        {
            double gain = covariance / raw_variance;
            double bias = real_mean - gain * raw_mean;

            /*
             * 运行保护峰值只约束实际校准区间内的拟合极值，不约束量程外推值。
             * 全 ADC 量程仅检查 float 可表示性，避免窄区间产生的异常斜率溢出。
             * 结果集中在此校验，通过后才窄化并更新缓存系数。
             */
            double fitted_min = (double)raw_min * gain + bias;
            double fitted_max = (double)raw_max * gain + bias;
            double full_scale_product = (double)ADC_FULL_SCALE_COUNT * gain;
            double full_scale_value = full_scale_product + bias;

            if(!(fabs(full_scale_product) <= (double)FLT_MAX
                 && fabs(bias) <= (double)FLT_MAX && fabs(full_scale_value) <= (double)FLT_MAX))
            {
                errors.numeric_range = 1U;
            }
            else if((float)gain <= 0.0f)
            {
                /* 同时拒绝窄化后下溢为零的增益。 */
                errors.gain_nonpositive = 1U;
            }
            else if(!(fabs(fitted_min) <= (double)s_value_limits[channel]
                      && fabs(fitted_max) <= (double)s_value_limits[channel]))
            {
                errors.value_range = 1U;
            }
            else
            {
                coefficient->gain = (float)gain;
                coefficient->bias = (float)bias;
            }
        }
    }

    return errors;
}
