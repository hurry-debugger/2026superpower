/** @file bsp_adc_feedback.h
 * @brief 双 ADC 单块循环 DMA、十次近似平均与五路反馈。
 */
#ifndef SUPERPOWER_BSP_ADC_FEEDBACK_H
#define SUPERPOWER_BSP_ADC_FEEDBACK_H

#include "adccalibration.h"
#include <stdbool.h>

typedef struct
{
    float adc_convert[ADC_CHANNEL_COUNT]; /**< AdcChannel_e 顺序，电压 V、电流 A。 */
    bool valid; /**< 数值有限且未报告采集故障，不承诺固定批次一致性或校准有效。 */
} BspAdcFeedback_t;

/** @brief 所属 .c 唯一写入；其他上下文读取时须按约定取得一致快照。 */
extern BspAdcFeedback_t g_bsp_adc_feedback;

/** @brief 每通道十次采样；250 kHz 触发对应 40 us，来源为当前采样配置。 */
#define BSP_ADC_FEEDBACK_SAMPLE_COUNT (10U)

/**
 * @brief 共同初始化采集状态并启动 ADC1[40]/ADC2[10] 循环 DMA。
 * @pre UID 匹配及两个 ADC 硬件校准已完成；HRTIM 未启动，功率输出关闭。
 * @note 仅启动阶段调用；关闭两路 HT 和 ADC2 TC，保留 ADC1 TC 及 DMA 错误中断。
 *       采集故障仅在停止计数器和输出后共同初始化清除；HAL 失败进入 Error_Handler。
 */
void BspAdcFeedback_Init(void);
/**
 * @brief ADC1 TC 中直接平均活动数组，换算并更新反馈，随后由同一上下文执行控制。
 * @pre ADC1 DMA 优先级 0，Master 优先级 1；其他读取者使用一致快照。
 * @note 允许读取期间窗口更新及两路窗口交叠，不配对批次、不等待首轮填满。
 *       采集故障后保留最近物理量并标记无效；不实现停更或控制超期检测。
 */
void BspAdcFeedback_Process(void);

#endif /* SUPERPOWER_BSP_ADC_FEEDBACK_H */
