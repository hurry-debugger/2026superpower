/**
 * @file power_ctrl_task.c
 * @brief ADC1 DMA 25 kHz 采样和控制，Master 1 kHz 快照后发送经典 CAN。
 */
#include "power_ctrl_task.h"

#include "adc.h"
#include "bsp_adc.h"
#include "bsp_usart.h"
#include "hrtim.h"
#include "power_ctrl.h"
#include "prot_supercap.h"

void PowerCtrlTask_Init(void)
{
    PowerCtrl_Init();
    BspAdcFeedback_Init();
    USER_UART_Init();
    SupercapCan_Init();
    __HAL_HRTIM_MASTER_CLEAR_IT(&hhrtim1, HRTIM_MASTER_IT_MREP);
    NVIC_ClearPendingIRQ(HRTIM1_Master_IRQn);
    if(HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER
        | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief ADC1 全满事件依次更新采样和控制，ADC2 不驱动正常控制。
 * @param[in] hadc 完成转换的 ADC 句柄。
 * @note ADC1 DMA 为优先级 0；两路循环 DMA 在处理期间继续采样。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc == &hadc1)
    {
        BspAdcFeedback_Process();
        PowerCtrl_Task();
    }
}

/**
 * @brief Master 1 kHz 重复中断取得反馈快照并发送经典 CAN。
 * @param[in] hhrtim 产生事件的 HRTIM 句柄。
 * @param[in] timer_index 产生重复事件的计数器索引。
 * @note Master 为优先级 1，允许 ADC1 DMA 抢占；不写控制状态。
 */
void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t timer_index)
{
    if(hhrtim == &hhrtim1 && timer_index == HRTIM_TIMERINDEX_MASTER)
        SupercapCan_SendStatus();
}
