/**
 * @file adccalibration.h
 * @note 文件归属 User/Calibration；引用者沿用同名头文件及新增包含路径。
 * @brief 超级电容板的理论换算、启动拟合、校准状态及统一物理量换算。
 * @details 参数依据用户提供的 Super-capacitor control.pdf 第 3～5 页，
 *          图纸 U7/U8 标为 INA240A2；用户于 2026-11-06 确认实装 INA240A1。
 *          A1 的 20 V/V 增益依据 TI 手册，采样精度仍待实板标定。
 *          ADC 满量程映射采用当前 ST LL 换算宏的 4095 分母，不代表消除了量化误差。
 * @note 不访问 ADC/DMA，不进行平均或低通；UID.c 负责匹配与发布全局状态。
 *       换算缓存只在启动初始化中更新，消费者启动后必须保持只读。
 */

#ifndef SUPERPOWER_ADCCALIBRATION_H
#define SUPERPOWER_ADCCALIBRATION_H

#include "adc_calibration_record.h"

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
/** @brief U7/U8 实装 INA240A1 的增益，单位 V/V；用户 2026-11-06 确认，TI 手册为 20。 */
#define ADC_CURRENT_SENSE_GAIN (20.0f)
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

/** @brief 底盘电压 ADC 理论量程上限，单位 V；由分压比和 ADC 满量程推导。 */
#define ADC_V_CHASSIS_MAX_V ((ADC_FULL_SCALE_COUNT - ADC_V_CHASSIS_OFFSET_COUNT) * ADC_V_CHASSIS_RATIO)
/** @brief 电池电压 ADC 理论量程上限，单位 V；由分压比和 ADC 满量程推导。 */
#define ADC_V_BAT_MAX_V ((ADC_FULL_SCALE_COUNT - ADC_V_BAT_OFFSET_COUNT) * ADC_V_BAT_RATIO)
/** @brief 电容电压 ADC 理论量程上限，单位 V；由分压比和 ADC 满量程推导。 */
#define ADC_V_CAP_MAX_V ((ADC_FULL_SCALE_COUNT - ADC_V_CAP_OFFSET_COUNT) * ADC_V_CAP_RATIO)
/** @brief 电池电流 ADC 理论量程下限，单位 A；由零点、采样电阻和放大倍数推导。 */
#define ADC_I_BAT_MIN_A (-ADC_I_BAT_OFFSET_COUNT * ADC_I_BAT_RATIO)
/** @brief 电池电流 ADC 理论量程上限，单位 A；由零点、采样电阻和放大倍数推导。 */
#define ADC_I_BAT_MAX_A ((ADC_FULL_SCALE_COUNT - ADC_I_BAT_OFFSET_COUNT) * ADC_I_BAT_RATIO)
/** @brief 底盘电流 ADC 理论量程下限，单位 A；由零点、采样电阻和放大倍数推导。 */
#define ADC_I_CHASSIS_MIN_A (-ADC_I_CHASSIS_OFFSET_COUNT * ADC_I_CHASSIS_RATIO)
/** @brief 底盘电流 ADC 理论量程上限，单位 A；由零点、采样电阻和放大倍数推导。 */
#define ADC_I_CHASSIS_MAX_A \
    ((ADC_FULL_SCALE_COUNT - ADC_I_CHASSIS_OFFSET_COUNT) * ADC_I_CHASSIS_RATIO)


/** @brief 所有通道均拟合有效的掩码，仅用于校准状态，不代表采样新鲜度。 */
#define ADC_CAL_ALL_CHANNELS_MASK ((1U << ADC_CHANNEL_COUNT) - 1U)

/* ==================== 系数及状态类型 ==================== */

/** @brief 统一线性换算的系数；物理量 = ADC 计数 * gain + bias。 */
typedef struct
{
    float gain; /**< 电压通道为 V/计数，电流通道为 A/计数。 */
    float bias; /**< 电压通道为 V，电流通道为 A，允许负值。 */
} AdcCalibrationCoeff_t;

/**
 * @brief 板卡识别错误标志；各成员 1 表示错误有效，0 表示无该错误。
 * @note 启动匹配后只读；仅通过成员访问，不依赖位域布局。
 */
typedef struct
{
    uint32_t no_record : 1; /**< 未登记板卡：当前 MCU UID 没有匹配记录。 */
    uint32_t duplicate_uid : 1; /**< 重复登记：当前 MCU UID 对应多个记录，拒绝任选一条。 */
    uint32_t invalid_id : 1; /**< 记录下标非法：初始化参数超出记录下标范围。 */
    uint32_t reserved : 29; /**< 保留，保持为 0。 */
} AdcCalibrationBoardError_t;

/**
 * @brief 每路校准输入和拟合错误；各成员 1 表示错误有效，0 表示无该错误。
 * @note 首个错误即停止本通道拟合，仅通过成员访问，不依赖位域布局。
 *       缺失和非有限输入由计数范围或实测值范围错误统一报告，不另存同义字段。
 */
typedef struct
{
    uint32_t count_range : 1; /**< 原始计数越界：不在 [0,4095]，包括 NaN 或无穷值。 */
    uint32_t zero_variance : 1; /**< 计数零方差：全部原始计数相同，无法拟合增益。 */
    uint32_t gain_nonpositive : 1; /**< 增益非正：拟合方向错误或窄化为 float 后下溢为零。 */
    uint32_t numeric_range : 1; /**< 数值溢出：系数或全量程换算相关值不可由 float 表示。 */
    uint32_t value_range : 1; /**< 物理量越界：实测值非法或校准区间拟合端点幅值超限。 */
    uint32_t reserved : 27; /**< 保留，保持为零；错误结构仍为一个 32 位字。 */
} AdcCalibrationChannelError_t;

/** @brief UID 匹配和启动拟合状态；全部通道有效且无板卡问题才允许控制。 */
typedef struct
{
    UID_t uid; /**< 实际 MCU UID，由 UID_MatchCalibration() 写入。 */
    SupercapId_e supercap_id; /**< 唯一匹配的记录下标，未识别时为 UNKNOWN。 */
    AdcCalibrationBoardError_t board_errors; /**< 板卡识别错误，按具名位域查看。 */
    AdcCalibrationChannelError_t channel_errors[ADC_CHANNEL_COUNT]; /**< 各通道独立拟合错误。 */
    uint32_t valid_channel_mask; /**< 通过拟合校验的通道位；不是 DMA 同步标志。 */
} AdcCalibrationState_t;

/**
 * @brief UID.c 中唯一定义，启动匹配函数发布后其他模块只读。
 * @note 未初始化时有效掩码为零，禁止启动控制；不能手动改掩码绕过校准。
 */
extern AdcCalibrationState_t g_adc_calibration_state;

/* ==================== 公共接口 ==================== */

/**
 * @brief 恢复理论系数，再对指定记录的五路原始数据分别拟合。
 * @param[in] supercap_id 唯一匹配的记录下标，UNKNOWN 表示无匹配记录。
 * @return 本次拟合状态；uid 暂为全零，由 UID.c 填入实际 UID 并发布。
 * @pre 仅在启动主线程、功率输出关闭且消费者尚未运行时调用。
 * @note 不读取硬件 UID，不直接修改全局状态；有效通道缓存拟合系数，其他通道
 *       保留理论值供观察。输入与校准区间拟合极值按 ADC 满量程、分压比、采样电阻、
 *       放大倍数和零点推导，不复用运行保护或控制目标上限。原始计数、正增益及可表示性检查保留。
 *       校准区间以外仅检查换算的 float 可表示性，不保证实板精度或允许误差。
 */
AdcCalibrationState_t AdcCalibration_Init(SupercapId_e supercap_id);

/**
 * @brief 使用显式增益和物理量偏置换算单个采样。
 * @param[in] raw_count 平均后的 ADC 计数，范围为 0～ADC_FULL_SCALE_COUNT。
 * @param[in] gain 增益，单位为 V/计数或 A/计数。
 * @param[in] bias 物理量偏置，单位为 V 或 A；不是原接口的计数零点。
 * @return raw_count * gain + bias，保留正负号。
 * @pre 两个系数均有限，运算中间值及结果可由 float 表示。
 * @note 不限幅、不重复扣零点、不检查参数；所有实现在 .c 中。
 */
float AdcCalibration_ConvertValue(uint16_t raw_count, float gain, float bias);

/**
 * @brief 按五路缓存系数换算平均计数，写入调用方提供的物理量数组。
 * @param[in] adc_raw 平均计数数组，各元素范围为 0～ADC_FULL_SCALE_COUNT。
 * @param[out] adc_convert 输出物理量数组；各索引的单位见 AdcChannel_e。
 * @return 传入的 adc_convert 地址；生命周期由调用方管理，下次写入前结果保持有效。
 * @pre 两个指针均非空，各有 ADC_CHANNEL_COUNT 个元素，输入输出内存不重叠。
 *      调用期间输入保持稳定、输出由本调用独占，系数缓存不允许同时初始化。
 * @note 未拟合或失败通道仍使用理论值供观察；返回地址不表示校准有效，控制启动
 *       必须检查全局状态。正常换算不匹配 UID、不拟合、不平均、不执行低通。
 *       电流正方向为采样电阻 INA240 的 IN+ 侧流向 IN- 侧，实际端口方向待上板验证。
 *       不保证采样同步性，不访问 DMA；可在主线程或中断使用，耗时须在目标板测量。
 */
float *AdcCalibration_Convert(const uint16_t adc_raw[ADC_CHANNEL_COUNT],
                              float adc_convert[ADC_CHANNEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* SUPERPOWER_ADCCALIBRATION_H */
