/** @file bsp_adc.h
 * @brief 双 ADC 单块循环 DMA、十次近似平均与五路反馈。
 */
#ifndef SUPERPOWER_BSP_ADC_H
#define SUPERPOWER_BSP_ADC_H

#include "adccalibration.h"

/** @brief ADC1 四个 Rank；来源为当前 ADC1 扫描配置。 */
#define BSP_ADC_FEEDBACK_ADC1_RANK_COUNT (4U)
#define BSP_ADC_FEEDBACK_ADC1_DMA_COUNT \
    (BSP_ADC_FEEDBACK_SAMPLE_COUNT * BSP_ADC_FEEDBACK_ADC1_RANK_COUNT)
    
typedef struct
{
    float adc_convert[ADC_CHANNEL_COUNT]; /**< AdcChannel_e 顺序，电压 V、电流 A。 */
    uint32_t adc_sum; /**< 当前通道十次采样计数和，0～40950；两路 ADC 顺序复用。 */
    uint16_t adc_raw[ADC_CHANNEL_COUNT]; /**< 五路平均计数，0～4095；仅采样上下文写入。 */
} BspAdcFeedback_t;

/** @brief 所属 .c 唯一写入；其他上下文读取时须按约定取得一致快照。 */
extern BspAdcFeedback_t g_bsp_adc_feedback;

/** @brief 每通道十次采样；250 kHz 触发对应 40 us，来源为当前采样配置。 */
#define BSP_ADC_FEEDBACK_SAMPLE_COUNT (10U)

/**
 * @brief 初始化反馈并启动 ADC1[40]/ADC2[10] 循环 DMA。
 * @pre UID 匹配及两个 ADC 硬件校准已完成；HRTIM 未启动，功率输出关闭。
 * @note 仅启动阶段调用；关闭两路 HT 和 ADC2 TC，保留 ADC1 TC 及 DMA 错误中断。
 *       HAL 启动失败进入 Error_Handler。
 */
void BspAdcFeedback_Init(void);
/**
 * @brief ADC1 TC 中直接平均活动数组，换算并更新反馈，随后由同一上下文执行控制。
 * @pre 仅由 ADC1 TC 调用，不可重入；ADC1 DMA 优先级 0，Master 优先级 1。
 *      adc_sum 与 adc_raw 仅由本采样上下文使用；其他读取者对物理量取得一致快照。
 * @note 允许读取期间窗口更新及两路窗口交叠，不配对批次、不等待首轮填满。
 *       不判断数值有效性或采集错误；不实现停更或控制超期检测。
 */
void BspAdcFeedback_Process(void);

#endif /* SUPERPOWER_BSP_ADC_H */
