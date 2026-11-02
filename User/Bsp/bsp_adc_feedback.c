/** @file bsp_adc_feedback.c
 * @brief 双 ADC 稳定块交接、十次平均及单写入者物理量发布。
 */
#include "bsp_adc_feedback.h"

#include "adc.h"
#include "filter.h"

#include <math.h>
#include <string.h>

/** @brief ADC1 四个 Rank；来源为当前 ADC1 扫描配置。 */
#define BSP_ADC_FEEDBACK_ADC1_RANK_COUNT (4U)
#define BSP_ADC_FEEDBACK_ADC1_HALF_COUNT \
    (BSP_ADC_FEEDBACK_SAMPLE_COUNT * BSP_ADC_FEEDBACK_ADC1_RANK_COUNT)

/** @brief 独立于硬件 DMA 环的稳定槽；先撤销有效标志，复制后才发布。 */
typedef struct
{
    uint16_t data[BSP_ADC_FEEDBACK_ADC1_HALF_COUNT];
    volatile uint32_t sequence;
    volatile bool valid;
} BspAdcFeedbackBlock_t;

/** @brief 元素总数各为两个半块；DMA 只写这里，消费者不直接平均活动缓冲。 */
static uint16_t s_adc1_dma[2U * BSP_ADC_FEEDBACK_ADC1_HALF_COUNT];
static uint16_t s_adc2_dma[2U * BSP_ADC_FEEDBACK_SAMPLE_COUNT];
static BspAdcFeedbackBlock_t s_blocks[2][2];
static uint32_t s_dma_sequence[2];
static uint32_t s_last_processed_sequence;
static bool s_next_full[2];
static volatile bool s_acquisition_fault;

static void BspAdcFeedback_Capture(ADC_HandleTypeDef *hadc, bool full);

BspAdcFeedback_t g_bsp_adc_feedback;

void BspAdcFeedback_Init(void)
{
    memset(&g_bsp_adc_feedback, 0, sizeof(g_bsp_adc_feedback));
    memset(s_blocks, 0, sizeof(s_blocks));
    memset(s_dma_sequence, 0, sizeof(s_dma_sequence));
    memset(s_next_full, 0, sizeof(s_next_full));
    s_last_processed_sequence = 0U;
    s_acquisition_fault = false;
    if(HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc1_dma,
                         2U * BSP_ADC_FEEDBACK_ADC1_HALF_COUNT) != HAL_OK
       || HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_adc2_dma,
                            2U * BSP_ADC_FEEDBACK_SAMPLE_COUNT) != HAL_OK)
    {
        Error_Handler();
    }
}

void BspAdcFeedback_Process(uint32_t now_ms)
{
    uint16_t adc_raw[ADC_CHANNEL_COUNT];
    float adc_convert[ADC_CHANNEL_COUNT];
    const BspAdcFeedbackBlock_t *adc1_block = NULL;
    const BspAdcFeedbackBlock_t *adc2_block = NULL;
    if(s_acquisition_fault)
    {
        BspAdcFeedback_Invalidate();
        return;
    }
    /* 两路相位不同，可由前一个完整槽配对，不能只比较各自最新序号。
     * Master 抢占 DMA 后，槽不会继续被软件写入；有效标志拒绝正在复制的槽。 */
    for(uint32_t slot = 0U; slot < 2U; slot++)
    {
        if(s_blocks[0][slot].valid && s_blocks[1][slot].valid)
        {
            __DMB();
            uint32_t sequence = s_blocks[0][slot].sequence;
            uint32_t advance = sequence - s_last_processed_sequence;
            if(sequence == s_blocks[1][slot].sequence && advance != 0U
               && advance < UINT32_MAX / 2U
               && (adc1_block == NULL
                   || advance > adc1_block->sequence - s_last_processed_sequence))
            {
                adc1_block = &s_blocks[0][slot];
                adc2_block = &s_blocks[1][slot];
            }
        }
    }
    if(adc1_block == NULL)
        return;
    for(uint32_t channel = 0U; channel < BSP_ADC_FEEDBACK_ADC1_RANK_COUNT; channel++)
        adc_raw[channel] = Filter_AverageU16(&adc1_block->data[channel],
            BSP_ADC_FEEDBACK_SAMPLE_COUNT, BSP_ADC_FEEDBACK_ADC1_RANK_COUNT);
    adc_raw[V_CAP] = Filter_AverageU16(adc2_block->data, BSP_ADC_FEEDBACK_SAMPLE_COUNT, 1U);
    AdcCalibration_Convert(adc_raw, adc_convert);
    s_last_processed_sequence = adc1_block->sequence;
    BspAdcFeedback_Publish(adc_convert, true, now_ms);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    BspAdcFeedback_Capture(hadc, false);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    BspAdcFeedback_Capture(hadc, true);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc == &hadc1 || hadc == &hadc2)
        s_acquisition_fault = true;
}

/**
 * @brief 在 HAL 已清除本次 HT/TC 标志后复制非活动半块。
 * @note 任意新边界标志都使复制失效，避免 Master 抢占期间 DMA 整圈回绕未被 CNDTR 识别。
 */
static void BspAdcFeedback_Capture(ADC_HandleTypeDef *hadc, bool full)
{
    uint32_t index;
    uint32_t count;
    const volatile uint16_t *source;
    if(hadc == &hadc1)
    {
        index = 0U;
        count = BSP_ADC_FEEDBACK_ADC1_HALF_COUNT;
        source = s_adc1_dma;
    }
    else if(hadc == &hadc2)
    {
        index = 1U;
        count = BSP_ADC_FEEDBACK_SAMPLE_COUNT;
        source = s_adc2_dma;
    }
    else
        return;
    if(s_acquisition_fault)
        return;

    DMA_HandleTypeDef *dma = hadc->DMA_Handle;
    uint32_t boundaries = __HAL_DMA_GET_HT_FLAG_INDEX(dma) | __HAL_DMA_GET_TC_FLAG_INDEX(dma);
    uint32_t remaining = __HAL_DMA_GET_COUNTER(dma);
    if(full != s_next_full[index] || __HAL_DMA_GET_FLAG(dma, boundaries) != RESET
       || remaining == 0U || remaining > 2U * count
       || (full ? remaining <= count : remaining > count))
    {
        s_acquisition_fault = true;
        return;
    }

    uint32_t sequence = s_dma_sequence[index] + 1U;
    BspAdcFeedbackBlock_t *block = &s_blocks[index][sequence % 2U];
    block->valid = false;
    __DMB();
    if(full)
        source += count;
    for(uint32_t i = 0U; i < count; i++)
        block->data[i] = source[i];
    __DMB();
    remaining = __HAL_DMA_GET_COUNTER(dma);
    if(__HAL_DMA_GET_FLAG(dma, boundaries) != RESET || remaining == 0U
       || remaining > 2U * count || (full ? remaining <= count : remaining > count))
    {
        s_acquisition_fault = true;
        return;
    }
    s_dma_sequence[index] = sequence;
    s_next_full[index] = !full;
    block->sequence = sequence;
    __DMB();
    block->valid = true;
}

void BspAdcFeedback_Publish(const float adc_convert[ADC_CHANNEL_COUNT], bool valid, uint32_t now_ms)
{
    g_bsp_adc_feedback.valid = false;
    for(uint32_t channel = 0U; channel < ADC_CHANNEL_COUNT; channel++)
    {
        g_bsp_adc_feedback.adc_convert[channel] = adc_convert[channel];
        if(!isfinite(adc_convert[channel]))
            valid = false;
    }
    g_bsp_adc_feedback.update_ms = now_ms;
    g_bsp_adc_feedback.sequence++;
    g_bsp_adc_feedback.valid = valid;
}

void BspAdcFeedback_Invalidate(void)
{
    g_bsp_adc_feedback.valid = false;
}
