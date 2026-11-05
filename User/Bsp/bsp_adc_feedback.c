/** @file bsp_adc_feedback.c
 * @brief 双 ADC 单块循环 DMA、十次近似平均及五路物理量发布。
 */
#include "bsp_adc_feedback.h"

#include "adc.h"

#include <math.h>
#include <string.h>

/** @brief ADC1 四个 Rank；来源为当前 ADC1 扫描配置。 */
#define BSP_ADC_FEEDBACK_ADC1_RANK_COUNT (4U)
#define BSP_ADC_FEEDBACK_ADC1_DMA_COUNT \
    (BSP_ADC_FEEDBACK_SAMPLE_COUNT * BSP_ADC_FEEDBACK_ADC1_RANK_COUNT)

/** @brief DMA 持续写入；CPU 逐次读取，允许平均期间采样窗口更新。 */
static volatile uint16_t s_adc1_dma[BSP_ADC_FEEDBACK_ADC1_DMA_COUNT];
static volatile uint16_t s_adc2_dma[BSP_ADC_FEEDBACK_SAMPLE_COUNT];
/** @brief 两路 ADC/DMA 的真实采集故障，仅共同初始化清除。 */
static volatile bool s_acquisition_fault;

BspAdcFeedback_t g_bsp_adc_feedback;

void BspAdcFeedback_Init(void)
{
    memset(&g_bsp_adc_feedback, 0, sizeof(g_bsp_adc_feedback));
    s_acquisition_fault = false;
    /* 地址转换仅满足 HAL 的 DMA 接口，CPU 平均仍通过 volatile 读取。 */
    if(HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc1_dma,
                         BSP_ADC_FEEDBACK_ADC1_DMA_COUNT) != HAL_OK
       || HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_adc2_dma,
                            BSP_ADC_FEEDBACK_SAMPLE_COUNT) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(hadc2.DMA_Handle, DMA_IT_HT | DMA_IT_TC);
}

void BspAdcFeedback_Process(void)
{
    /* ADC 专用 IRQ 未接入；直接读取 OVR，不能将活动数组更新视为故障。 */
    if(__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR) != RESET
       || __HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_OVR) != RESET)
        s_acquisition_fault = true;
    g_bsp_adc_feedback.valid = false;
    if(!s_acquisition_fault)
    {
        uint16_t adc_raw[ADC_CHANNEL_COUNT];
        bool valid = true;
        for(uint32_t channel = 0U; channel < BSP_ADC_FEEDBACK_ADC1_RANK_COUNT; channel++)
        {
            uint32_t sum = 0U;
            for(uint32_t sample = 0U; sample < BSP_ADC_FEEDBACK_SAMPLE_COUNT; sample++)
                sum += s_adc1_dma[sample * BSP_ADC_FEEDBACK_ADC1_RANK_COUNT + channel];
            adc_raw[channel] = (uint16_t)((sum + BSP_ADC_FEEDBACK_SAMPLE_COUNT / 2U)
                / BSP_ADC_FEEDBACK_SAMPLE_COUNT);
        }
        uint32_t sum = 0U;
        for(uint32_t sample = 0U; sample < BSP_ADC_FEEDBACK_SAMPLE_COUNT; sample++)
            sum += s_adc2_dma[sample];
        adc_raw[V_CAP] = (uint16_t)((sum + BSP_ADC_FEEDBACK_SAMPLE_COUNT / 2U)
            / BSP_ADC_FEEDBACK_SAMPLE_COUNT);
        AdcCalibration_Convert(adc_raw, g_bsp_adc_feedback.adc_convert);
        for(uint32_t channel = 0U; channel < ADC_CHANNEL_COUNT; channel++)
            if(!isfinite(g_bsp_adc_feedback.adc_convert[channel]))
                valid = false;
        g_bsp_adc_feedback.valid = valid;
    }
}

/**
 * @brief 锁存两路 ADC/DMA 的真实错误并标记反馈无效，不自动重启采集。
 * @param[in] hadc HAL 报告错误的 ADC 句柄。
 * @note 本回调不操作控制状态或输出；没有后续 ADC1 TC 时不保证软件关断。
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc == &hadc1 || hadc == &hadc2)
    {
        s_acquisition_fault = true;
        g_bsp_adc_feedback.valid = false;
    }
}
