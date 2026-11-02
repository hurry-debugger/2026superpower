/**
 * @file adc_calibration_record.c
 * @brief 直接在编辑器中填写每块超电的 UID、ADC 平均计数及仪表实测值。
 * @note 当前只有待填写模板，无真实校准数据；因此默认不允许启动功率输出。
 *       从全局状态 uid 读取当前芯片 UID，填入对应槽位；同列填写配对采集值。
 *       更换 MCU 后重新绑定；更换采样电阻或放大器等模拟前端后重新校准。
 */

#include "adc_calibration_record.h"

#include <math.h>

const AdcCalibrationRecord_t g_adc_calibration_records[SUPERCAP_COUNT] =
{
    [SUPERCAP_ID_01] =
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
