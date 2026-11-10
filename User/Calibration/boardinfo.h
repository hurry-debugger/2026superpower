/**
 * @file boardinfo.h
 * @brief 板卡主要信息及启动识别、ADC 校准初始化入口。
 */
#ifndef SUPERPOWER_BOARDINFO_H
#define SUPERPOWER_BOARDINFO_H

#include "adc_calibration_record.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 板卡识别错误标志；各成员 1 表示错误有效，0 表示无该错误。
 * @note 启动匹配后只读；仅通过成员访问，不依赖位域布局。
 */
typedef struct
{
    uint32_t no_record : 1; /**< 未登记板卡：当前 MCU UID 没有匹配记录。 */
    uint32_t duplicate_uid : 1; /**< 重复登记：当前 MCU UID 对应多个记录，拒绝任选一条。 */
    uint32_t reserved : 30; /**< 保留，保持为 0。 */
} BoardError_t;

/** @brief 板卡主要信息：芯片 UID、板卡 ID、识别错误及 ADC 校准结果。 */
typedef struct
{
    UID_t uid; /**< 实际 MCU UID，由 BoardInfo_Init() 调用 UID_Read() 写入。 */
    SupercapId_e supercap_id; /**< 唯一匹配的记录下标，未识别时为 UNKNOWN。 */
    BoardError_t board_errors; /**< 板卡识别错误，按具名位域查看。 */
    AdcCalibrationChannelError_t channel_errors[ADC_CHANNEL_COUNT]; /**< 各通道独立拟合错误。 */
    uint32_t uncalibrated_channel_mask; /**< 对应 AdcChannel_e 的位为 1 表示未校准；非零表示存在未校准通道。 */
} BoardInfo_t;

/**
 * @brief boardinfo.c 中唯一定义；启动初始化直接填写，消费者运行后只读。
 * @note 初始化时标记全部通道未校准，成功通道逐位清除；仅供诊断，不参与控制错误汇总。
 *       不手动修改诊断结果；未校准通道沿用默认增益和偏置，其他保护条件仍生效。
 */
extern BoardInfo_t g_board_info;

/**
 * @brief 读取 UID、唯一匹配板卡记录，并逐通道拟合、汇总板卡信息。
 * @pre 仅在启动主线程调用一次，功率输出关闭，换算和状态消费者尚未运行。
 * @note 跳过全零 UID 槽位；无匹配或重复匹配不执行拟合，全部通道保持未校准。
 *       拟合成功清除对应未校准位；失败保留该位及默认系数，不控制 HRTIM 输出。
 */
void BoardInfo_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* SUPERPOWER_BOARDINFO_H */
