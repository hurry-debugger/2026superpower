/**
 * @file adc_calibration_record.c
 * @brief 保存手填采集记录，启动时拟合 ADC 系数并提供运行期物理量换算。
 * @note 当前只有待填写模板，无真实校准数据；因此默认不允许启动功率输出。
 *       从全局状态 uid 读取当前芯片 UID，填入对应槽位；同列填写配对采集值。
 *       更换 MCU 后重新绑定；更换采样电阻或放大器等模拟前端后重新校准。
 */

#include "adc_calibration_record.h"

#include <float.h>
#include <math.h>

const AdcCalibrationRecord_t g_adc_calibration_records[SUPERCAP_COUNT] =
{

    [SUPERCAP_ID_01] =
    {
        .uid =
        {
            .word = {0x00200046, 0x48525010, 0x20323733}
        },
        .calibration_data =
        {
            [ADC_CAL_RAW_ROW(V_CHASSIS)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_REAL_ROW(V_CHASSIS)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_RAW_ROW(V_BAT)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_REAL_ROW(V_BAT)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_RAW_ROW(I_BAT)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_REAL_ROW(I_BAT)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_RAW_ROW(I_CHASSIS)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_REAL_ROW(I_CHASSIS)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_RAW_ROW(V_CAP)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_REAL_ROW(V_CAP)] = {NAN, NAN, NAN, NAN, NAN}
        }
    },
    [SUPERCAP_ID_02] =
    {
        .uid =
        {
            .word = {0U, 0U, 0U}
        },
        .calibration_data =
        {
            [ADC_CAL_RAW_ROW(V_CHASSIS)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_REAL_ROW(V_CHASSIS)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_RAW_ROW(V_BAT)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_REAL_ROW(V_BAT)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_RAW_ROW(I_BAT)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_REAL_ROW(I_BAT)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_RAW_ROW(I_CHASSIS)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_REAL_ROW(I_CHASSIS)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_RAW_ROW(V_CAP)] = {NAN, NAN, NAN, NAN, NAN},
            [ADC_CAL_REAL_ROW(V_CAP)] = {NAN, NAN, NAN, NAN, NAN}
        }
    }
};

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

/** @brief 校准实测值理论下限 V/A，由 ADC 模拟链参数推导。 */
static const float s_value_minimums[ADC_CHANNEL_COUNT] =
{
    [V_CHASSIS] = 0.0f,
    [V_BAT] = 0.0f,
    [I_BAT] = ADC_I_BAT_MIN_A,
    [I_CHASSIS] = ADC_I_CHASSIS_MIN_A,
    [V_CAP] = 0.0f
};

/** @brief 校准实测值理论上限 V/A，由 ADC 模拟链参数推导。 */
static const float s_value_maximums[ADC_CHANNEL_COUNT] =
{
    [V_CHASSIS] = ADC_V_CHASSIS_MAX_V,
    [V_BAT] = ADC_V_BAT_MAX_V,
    [I_BAT] = ADC_I_BAT_MAX_A,
    [I_CHASSIS] = ADC_I_CHASSIS_MAX_A,
    [V_CAP] = ADC_V_CAP_MAX_V
};

/* ==================== 公共接口实现 ==================== */

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

AdcCalibrationChannelError_t AdcCalibration_FitChannel(SupercapId_e supercap_id,
                                                      AdcChannel_e channel)
{
    /* 返回本次拟合错误；失败前不能复用或覆盖调用方已有的换算系数。 */
    AdcCalibrationChannelError_t errors = {0};
    const float *raw = g_adc_calibration_records[supercap_id].calibration_data[ADC_CAL_RAW_ROW(channel)];
    const float *real = g_adc_calibration_records[supercap_id].calibration_data[ADC_CAL_REAL_ROW(channel)];
    const float min_value = s_value_minimums[channel];
    const float max_value = s_value_maximums[channel];
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
        else if(!(real[point] >= min_value && real[point] <= max_value))
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
             * 校准区间边界由 ADC 模拟链理论量程约束，不复用运行保护或控制目标上限。
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
            else if(!(fitted_min >= (double)min_value && fitted_max <= (double)max_value))
            {
                errors.value_range = 1U;
            }
            else
            {
                s_coefficients[channel].gain = (float)gain;
                s_coefficients[channel].bias = (float)bias;
            }
        }
    }

    return errors;
}
