/** @file bsp_adc.c
 * @brief 双 ADC 单块循环 DMA、十次近似平均及五路物理量发布。
 */
#include "bsp_adc.h"
#include "adc.h"
#include <string.h>

/** @brief DMA 持续写入；CPU 逐次读取，允许平均期间采样窗口更新。 */
static volatile uint16_t s_adc1_dma[BSP_ADC_FEEDBACK_ADC1_DMA_COUNT];
static volatile uint16_t s_adc2_dma[BSP_ADC_FEEDBACK_SAMPLE_COUNT];

BspAdcFeedback_t g_bsp_adc_feedback;

void BspAdcFeedback_Init(void)
{
    memset(&g_bsp_adc_feedback, 0, sizeof(g_bsp_adc_feedback));
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
    for(uint32_t channel = 0U; channel < BSP_ADC_FEEDBACK_ADC1_RANK_COUNT; channel++)
    {
        g_bsp_adc_feedback.adc_sum = 0U;
        for(uint32_t sample = 0U; sample < BSP_ADC_FEEDBACK_SAMPLE_COUNT; sample++)
        {
            g_bsp_adc_feedback.adc_sum +=
                s_adc1_dma[sample * BSP_ADC_FEEDBACK_ADC1_RANK_COUNT + channel];
        }
        g_bsp_adc_feedback.adc_raw[channel] = (uint16_t)(
            (g_bsp_adc_feedback.adc_sum + BSP_ADC_FEEDBACK_SAMPLE_COUNT / 2U)
            / BSP_ADC_FEEDBACK_SAMPLE_COUNT);
    }
    g_bsp_adc_feedback.adc_sum = 0U;
    for(uint32_t sample = 0U; sample < BSP_ADC_FEEDBACK_SAMPLE_COUNT; sample++)
    {
        g_bsp_adc_feedback.adc_sum += s_adc2_dma[sample];
    }
    g_bsp_adc_feedback.adc_raw[V_CAP] = (uint16_t)(
        (g_bsp_adc_feedback.adc_sum + BSP_ADC_FEEDBACK_SAMPLE_COUNT / 2U)
        / BSP_ADC_FEEDBACK_SAMPLE_COUNT);
    AdcCalibration_Convert(g_bsp_adc_feedback.adc_raw, g_bsp_adc_feedback.adc_convert);
}
