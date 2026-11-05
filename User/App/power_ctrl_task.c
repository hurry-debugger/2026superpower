/**
 * @file power_ctrl_task.c
 * @brief Master 采样和控制，Timer B 快照后发送经典 CAN。
 */
#include "power_ctrl_task.h"

#include "bsp_adc_feedback.h"
#include "bsp_usart.h"
#include "hrtim.h"
#include "power_ctrl.h"
#include "prot_supercap.h"

#include <string.h>

PowerCtrlTask_t g_power_ctrl_task;

static void PowerCtrlTask_SendCan(void);

void PowerCtrlTask_Init(void)
{
    memset(&g_power_ctrl_task, 0, sizeof(g_power_ctrl_task));
    PowerCtrl_Init();
    BspAdcFeedback_Init();
    USER_UART_Init();
    SupercapCan_Init();
    __HAL_HRTIM_MASTER_CLEAR_IT(&hhrtim1, HRTIM_MASTER_IT_MREP);
    __HAL_HRTIM_TIMER_CLEAR_IT(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_TIM_IT_REP);
    NVIC_ClearPendingIRQ(HRTIM1_Master_IRQn);
    NVIC_ClearPendingIRQ(HRTIM1_TIMB_IRQn);
    if(HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER
        | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t timer_index)
{
    if(hhrtim != &hhrtim1)
        return;
    if(timer_index == HRTIM_TIMERINDEX_MASTER)
    {
        uint32_t now_ms = HAL_GetTick();

        BspAdcFeedback_Process(now_ms);
        PowerCtrl_Task();
        /* HAL 已在进入回调前清除本次标志，再置位说明一次 40 us 期限已经错过。 */
        if(__HAL_HRTIM_MASTER_GET_FLAG(hhrtim, HRTIM_MASTER_FLAG_MREP) != RESET)
        {
            g_power_ctrl_task.master_overrun_count++;
            PowerCtrl_NotifyTimingFault();
            __HAL_HRTIM_MASTER_CLEAR_IT(hhrtim, HRTIM_MASTER_IT_MREP);
            NVIC_ClearPendingIRQ(HRTIM1_Master_IRQn);
        }
    }
    else if(timer_index == HRTIM_TIMERINDEX_TIMER_B)
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
