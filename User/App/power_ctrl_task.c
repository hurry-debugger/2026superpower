/**
 * @file power_ctrl_task.c
 * @brief ADC1 DMA 25 kHz 采样和控制，Master 1 kHz 快照后发送经典 CAN。
 */
#include "power_ctrl_task.h"

#include "adc.h"
#include "bsp_adc_feedback.h"
#include "bsp_usart.h"
#include "hrtim.h"
#include "power_ctrl.h"
#include "prot_supercap.h"

static void PowerCtrlTask_SendCan(void);

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
        PowerCtrlTask_SendCan();
}

static void PowerCtrlTask_SendCan(void)
{
    float cap_voltage_v;
    float chassis_current_a;
    uint32_t errors;
    PowerCtrlState_e state;
    uint8_t faults = 0U;
    uint8_t wire_state = SUPERCAP_CAN_STATE_PROTECT;

    /* 仅复制最小反馈，恢复调用前 PRIMASK；编码与 HAL 发送均在临界区之外。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    cap_voltage_v = g_bsp_adc_feedback.adc_convert[V_CAP];
    chassis_current_a = g_bsp_adc_feedback.adc_convert[I_CHASSIS];
    errors = g_power_ctrl.errors;
    state = g_power_ctrl.state;
    __DMB();
    __set_PRIMASK(primask);

    if((errors & POWER_CTRL_ERROR_CAP_OVERVOLTAGE) != 0U)
        faults |= SUPERCAP_CAN_FAULT_CAP_OVERVOLTAGE;
    if((errors & POWER_CTRL_ERROR_CHASSIS_OVERVOLTAGE) != 0U)
        faults |= SUPERCAP_CAN_FAULT_CHASSIS_OVERVOLTAGE;
    if((errors & POWER_CTRL_ERROR_BAT_UNDERVOLTAGE) != 0U)
        faults |= SUPERCAP_CAN_FAULT_BAT_UNDERVOLTAGE;
    if((errors & POWER_CTRL_ERROR_REFEREE) != 0U)
        faults |= SUPERCAP_CAN_FAULT_REFEREE;
    if(state == CTRL_MODE)
        wire_state = SUPERCAP_CAN_STATE_NORMAL;
    else if(state == RESTART_MODE)
        wire_state = SUPERCAP_CAN_STATE_RESTART;
    SupercapCan_SendFeedback(cap_voltage_v, chassis_current_a, faults, wire_state);
}
