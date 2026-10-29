/**
 * @file adc_calibration_record.h
 * @brief 超电 ID、五路通道及手填校准记录的数据布局。
 * @note 本文件只定义类型、宏及声明；数据在对应 .c 中初始化。
 */

#ifndef SUPERPOWER_ADC_CALIBRATION_RECORD_H
#define SUPERPOWER_ADC_CALIBRATION_RECORD_H

#include "UID.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief 每路固定的配对校准点数；统一调整后须同步填写每一行。 */
#define ADC_CAL_POINT_COUNT (5U)
/** @brief 每通道两行：ADC 计数、实测物理量。 */
#define ADC_CAL_FIELD_COUNT (2U)
/** @brief 通道对应的 ADC 原始平均计数行，无单位；允许小数。 */
#define ADC_CAL_RAW_ROW(channel) (2U * (channel))
/** @brief 通道对应的实测值行；电压为 V，电流为带符号的 A。 */
#define ADC_CAL_REAL_ROW(channel) (2U * (channel) + 1U)

#if ADC_CAL_POINT_COUNT < 2U
#error ADC_CAL_POINT_COUNT_must_be_at_least_two
#endif

/** @brief 记录下标即超电 ID；01 是易读标签，枚举值 0 是数组下标。 */
typedef enum
{
    SUPERCAP_ID_UNKNOWN = -1, /**< 未识别，不得用于数组索引。 */
    SUPERCAP_ID_01 = 0,       /**< 第一个记录槽位，当前 UID 和采集点待填写。 */
    SUPERCAP_COUNT           /**< 记录槽位数；新增 ID 必须放在它之前。 */
} SupercapId_e;

/** @brief adc_raw、adc_convert 和校准记录共用的五路通道索引。 */
typedef enum
{
    V_CHASSIS = 0,    /**< 底盘电压 V，ADC1 Rank 1 / IN1 / PA0。 */
    V_BAT,            /**< 电池电压 V，ADC1 Rank 2 / IN3 / PA2。 */
    I_BAT,            /**< 电池电流 A，ADC1 Rank 3 / IN5 / PB14。 */
    I_CHASSIS,        /**< 底盘电流 A，ADC1 Rank 4 / IN12 / PB1。 */
    V_CAP,            /**< 电容电压 V，ADC2 Rank 1 / IN2 / PA1，生成标签 V_CUP。 */
    ADC_CHANNEL_COUNT /**< 数组元素数，不是有效通道。 */
} AdcChannel_e;

/**
 * @brief 每块超电的完整 UID 与原始校准记录，严格保留两个成员。
 * @note 同列的 ADC 计数和实测值配对；每路必须填满固定点数。
 *       缺失位置显式填 NAN，不得省略元素让 C 自动补零；合法零值不能表示缺失。
 */
typedef struct
{
    UID_t uid; /**< 全零为未登记槽位，不参与 UID 匹配。 */
    float calibration_data[ADC_CHANNEL_COUNT * ADC_CAL_FIELD_COUNT][ADC_CAL_POINT_COUNT];
    /**< 偶数行为 ADC 平均计数 [0,4095]，奇数行为对应 V/A；通过行宏索引。 */
} AdcCalibrationRecord_t;

/** @brief 在 .c 中按超电 ID 指定下标初始化；固件运行期间只读。 */
extern const AdcCalibrationRecord_t g_adc_calibration_records[SUPERCAP_COUNT];

#ifdef __cplusplus
}
#endif

#endif /* SUPERPOWER_ADC_CALIBRATION_RECORD_H */
