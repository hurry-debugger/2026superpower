/**
 * @file myadc.h
 * @brief 超级电容板的 ADC 通道、标称参数及物理量换算。
 * @details 参数依据用户提供的 Super-capacitor control.pdf 第 3～5 页，
 *          INA240A2 增益同时核对 TI 手册；图纸与实板一致性及标定仍待验证。
 *          ADC 满量程映射采用当前 ST LL 换算宏的 4095 分母，不代表消除了量化误差。
 * @note 不访问 ADC/DMA，不进行平均或低通；调用方提供平均后的稳定计数数组。
 */

#ifndef SUPERPOWER_MYADC_H
#define SUPERPOWER_MYADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ==================== 标称硬件参数 ==================== */

/** @brief ADC 的 VREF+ 标称电压，单位 V；原理图 U5 REF3033，实测值待标定。 */
#define ADC_REFERENCE_V (3.3f)
/** @brief 12 位右对齐 ADC 的满量程计数，不启用硬件过采样。 */
#define ADC_FULL_SCALE_COUNT (4095U)
/** @brief 底盘分压还原倍数，无量纲；R26=13 kΩ、R34=1 kΩ。 */
#define ADC_V_CHASSIS_DIVIDER_RATIO (14.0f)
/** @brief 电池分压还原倍数，无量纲；R38=10 kΩ、R46=1 kΩ。 */
#define ADC_V_BAT_DIVIDER_RATIO (11.0f)
/** @brief 电容分压还原倍数，无量纲；R27=10 kΩ、R71=1 kΩ。 */
#define ADC_V_CAP_DIVIDER_RATIO (11.0f)
/** @brief U7/U8 INA240A2 的标称电压增益，单位 V/V。 */
#define ADC_CURRENT_SENSE_GAIN (50.0f)
/** @brief R3/R18 电流采样电阻，单位 Ω。 */
#define ADC_CURRENT_SHUNT_OHM (0.003f)
/** @brief 电池电流通道零电流输出，单位 V；U9 参考的一半，独立于 ADC 的 U5 参考。 */
#define ADC_I_BAT_ZERO_V (1.65f)
/** @brief 底盘电流通道零电流输出，单位 V；标称值，后续可独立标定。 */
#define ADC_I_CHASSIS_ZERO_V (1.65f)

/* ==================== 各通道转换参数 ==================== */

/** @brief 底盘电压转换系数，单位 V/计数。 */
#define ADC_V_CHASSIS_RATIO \
    (ADC_REFERENCE_V * ADC_V_CHASSIS_DIVIDER_RATIO / ADC_FULL_SCALE_COUNT)
/** @brief 电池电压转换系数，单位 V/计数。 */
#define ADC_V_BAT_RATIO (ADC_REFERENCE_V * ADC_V_BAT_DIVIDER_RATIO / ADC_FULL_SCALE_COUNT)
/** @brief 电容电压转换系数，单位 V/计数。 */
#define ADC_V_CAP_RATIO (ADC_REFERENCE_V * ADC_V_CAP_DIVIDER_RATIO / ADC_FULL_SCALE_COUNT)
/** @brief 电池电流转换系数，单位 A/计数。 */
#define ADC_I_BAT_RATIO \
    (ADC_REFERENCE_V / ADC_FULL_SCALE_COUNT / (ADC_CURRENT_SENSE_GAIN * ADC_CURRENT_SHUNT_OHM))
/** @brief 底盘电流转换系数，单位 A/计数。 */
#define ADC_I_CHASSIS_RATIO \
    (ADC_REFERENCE_V / ADC_FULL_SCALE_COUNT / (ADC_CURRENT_SENSE_GAIN * ADC_CURRENT_SHUNT_OHM))

/** @brief 底盘电压零点，单位计数；标称无偏置。 */
#define ADC_V_CHASSIS_OFFSET_COUNT (0.0f)
/** @brief 电池电压零点，单位计数；标称无偏置。 */
#define ADC_V_BAT_OFFSET_COUNT (0.0f)
/** @brief 电容电压零点，单位计数；标称无偏置。 */
#define ADC_V_CAP_OFFSET_COUNT (0.0f)
/** @brief 电池电流零点，单位计数；标称 2047.5，不提前取整。 */
#define ADC_I_BAT_OFFSET_COUNT (ADC_I_BAT_ZERO_V * ADC_FULL_SCALE_COUNT / ADC_REFERENCE_V)
/** @brief 底盘电流零点，单位计数；允许与电池通道分别标定。 */
#define ADC_I_CHASSIS_OFFSET_COUNT (ADC_I_CHASSIS_ZERO_V * ADC_FULL_SCALE_COUNT / ADC_REFERENCE_V)

/* ==================== 通道定义 ==================== */

/** @brief adc_raw 与 adc_convert 共用的数组索引；成员注释给出转换后单位。 */
typedef enum
{
    V_CHASSIS = 0,    /**< 底盘电压 V，ADC1 Rank 1 / IN1 / PA0。 */
    V_BAT,            /**< 电池电压 V，ADC1 Rank 2 / IN3 / PA2。 */
    I_BAT,            /**< 电池电流 A，ADC1 Rank 3 / IN5 / PB14。 */
    I_CHASSIS,        /**< 底盘电流 A，ADC1 Rank 4 / IN12 / PB1。 */
    V_CAP,            /**< 电容电压 V，ADC2 Rank 1 / IN2 / PA1，生成标签 V_CUP。 */
    ADC_CHANNEL_COUNT /**< 数组元素数，不是有效通道。 */
} AdcChannel_e;

/* ==================== 公共接口 ==================== */

/**
 * @brief 按指定系数和原始计数零点，将单个采样转换为物理量。
 * @param[in] raw_count 平均后的 ADC 计数，范围为 0～ADC_FULL_SCALE_COUNT。
 * @param[in] ratio 转换系数，单位为输出物理量/计数，例如 V/计数或 A/计数。
 * @param[in] offset_count 零点，单位计数；允许保留小数。
 * @return (raw_count - offset_count) * ratio，单位由 ratio 决定，保留正负号。
 * @pre ratio、offset_count 均有限，运算中间值及结果可由 float 表示。
 * @note 不限幅、不自动调零、不检查参数；实现在 myadc.c，调用优化由编译器决定。
 */
float Myadc_ConvertValue(uint16_t raw_count, float ratio, float offset_count);

/**
 * @brief 按通道分别换算平均后的原始计数，写入调用方提供的物理量数组。
 * @param[in] adc_raw 平均计数数组，各元素范围为 0～ADC_FULL_SCALE_COUNT。
 * @param[out] adc_convert 输出物理量数组；各索引的单位见 AdcChannel_e。
 * @return 传入的 adc_convert 地址；生命周期由调用方管理，下次写入前结果保持有效。
 * @pre 两个指针均非空，各有 ADC_CHANNEL_COUNT 个元素，输入输出内存不重叠。
 *      调用期间输入保持稳定、输出由本调用独占，宏参数须满足通用换算函数的数值约束。
 * @note 不持有内部状态，可用于独立数组；不检查参数、不返回函数状态。
 *       电流正方向为采样电阻 INA240 的 IN+ 侧流向 IN- 侧，实际端口方向待上板验证。
 *       不保证采样同步性，不访问 DMA；可在主线程或中断使用，耗时须在目标板测量。
 */
float *Myadc_Convert(const uint16_t adc_raw[ADC_CHANNEL_COUNT],
                     float adc_convert[ADC_CHANNEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* SUPERPOWER_MYADC_H */
