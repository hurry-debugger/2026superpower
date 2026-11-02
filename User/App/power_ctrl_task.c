/**
 * @file power_ctrl_task.c
 * @brief Master 串行解析、采样和控制，Timer B 快照后发送经典 CAN。
 */
#include "power_ctrl_task.h"

#include "bsp_adc_feedback.h"
#include "bsp_usart.h"
#include "fdcan.h"
#include "hrtim.h"
#include "power_ctrl.h"
#include "power_ctrl_config.h"
#include "prot_supercap.h"
#include "usart.h"

#include <string.h>

/** @brief 本轮批准的 NVIC 抢占优先级，数值越小优先级越高。 */
#define POWER_CTRL_TASK_MASTER_PRIORITY (0U)
#define POWER_CTRL_TASK_ADC_PRIORITY (1U)
#define POWER_CTRL_TASK_CAN_TX_PRIORITY (2U)
#define POWER_CTRL_TASK_COMM_PRIORITY (3U)
/** @brief 当前 170 MHz/MUL32/21760 对应 PWM 250 kHz，配置来源为 CubeMX。 */
#define POWER_CTRL_TASK_HRTIM_CLOCK_HZ (170000000U)
#define POWER_CTRL_TASK_PWM_PERIOD_COUNT (21760U)
#define POWER_CTRL_TASK_MASTER_REPETITION (9U)
#define POWER_CTRL_TASK_CAN_REPETITION (249U)

PowerCtrlTask_t g_power_ctrl_task;

static bool PowerCtrlTask_ConfigValid(void);
static void PowerCtrlTask_SendCan(void);

void PowerCtrlTask_Init(void)
{
    memset(&g_power_ctrl_task, 0, sizeof(g_power_ctrl_task));
    PowerCtrl_Init();
    if(!PowerCtrlTask_ConfigValid())
    {
        /* 不绕过缺失的 CubeMX 配置；保持输出关闭，供调试器观察错误。 */
        PowerCtrl_Task();
        return;
    }
    BspAdcFeedback_Init();
    USER_UART_Init(&huart1);
    SupercapCan_Init();
    __HAL_HRTIM_MASTER_CLEAR_IT(&hhrtim1, HRTIM_MASTER_IT_MREP);
    __HAL_HRTIM_TIMER_CLEAR_IT(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_TIM_IT_REP);
    NVIC_ClearPendingIRQ(HRTIM1_Master_IRQn);
    NVIC_ClearPendingIRQ(HRTIM1_TIMB_IRQn);
    g_power_ctrl_task.configuration_ready = true;
    if(HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER
        | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B) != HAL_OK)
    {
        g_power_ctrl_task.configuration_ready = false;
        Error_Handler();
    }
}

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t timer_index)
{
    if(hhrtim != &hhrtim1 || !g_power_ctrl_task.configuration_ready)
        return;
    if(timer_index == HRTIM_TIMERINDEX_MASTER)
    {
        uint32_t now_ms = HAL_GetTick();
        USER_UART_Process(now_ms);
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

static bool PowerCtrlTask_ConfigValid(void)
{
    if(HAL_RCC_GetHCLKFreq() != POWER_CTRL_TASK_HRTIM_CLOCK_HZ
       || hhrtim1.Instance->sMasterRegs.MPER != POWER_CTRL_TASK_PWM_PERIOD_COUNT
       || hhrtim1.Instance->sMasterRegs.MREP != POWER_CTRL_TASK_MASTER_REPETITION
       || (hhrtim1.Instance->sMasterRegs.MCR & HRTIM_MCR_CK_PSC) != HRTIM_PRESCALERRATIO_MUL32
       || hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].REPxR
            != POWER_CTRL_TASK_CAN_REPETITION
       || hhrtim1.TimerParam[HRTIM_TIMERINDEX_MASTER].InterruptRequests != HRTIM_MASTER_IT_MREP
       || hhrtim1.TimerParam[HRTIM_TIMERINDEX_TIMER_A].InterruptRequests != HRTIM_TIM_IT_NONE
       || hhrtim1.TimerParam[HRTIM_TIMERINDEX_TIMER_B].InterruptRequests != HRTIM_TIM_IT_REP
       || hfdcan1.Init.StdFiltersNbr < 1U)
        return false;
    for(uint32_t timer = HRTIM_TIMERINDEX_TIMER_A; timer <= HRTIM_TIMERINDEX_TIMER_B; timer++)
    {
        uint32_t control = hhrtim1.Instance->sTimerxRegs[timer].TIMxCR;
        if(hhrtim1.Instance->sTimerxRegs[timer].PERxR != POWER_CTRL_TASK_PWM_PERIOD_COUNT
           || (control & HRTIM_TIMCR_CK_PSC) != HRTIM_PRESCALERRATIO_MUL32
           || (control & (HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TRSTU))
                != (HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TRSTU)
           || (control & HRTIM_TIMCR_TREPU) != 0U)
            return false;
    }
    return NVIC_GetPriority(HRTIM1_Master_IRQn) == POWER_CTRL_TASK_MASTER_PRIORITY
        && NVIC_GetPriority(DMA1_Channel1_IRQn) == POWER_CTRL_TASK_ADC_PRIORITY
        && NVIC_GetPriority(DMA1_Channel2_IRQn) == POWER_CTRL_TASK_ADC_PRIORITY
        && NVIC_GetPriority(HRTIM1_TIMB_IRQn) == POWER_CTRL_TASK_CAN_TX_PRIORITY
        && NVIC_GetPriority(USART1_IRQn) == POWER_CTRL_TASK_COMM_PRIORITY
        && NVIC_GetPriority(DMA1_Channel3_IRQn) == POWER_CTRL_TASK_COMM_PRIORITY
        && NVIC_GetPriority(DMA1_Channel4_IRQn) == POWER_CTRL_TASK_COMM_PRIORITY
        && NVIC_GetPriority(FDCAN1_IT0_IRQn) == POWER_CTRL_TASK_COMM_PRIORITY
        && NVIC_GetPriority(FDCAN1_IT1_IRQn) == POWER_CTRL_TASK_COMM_PRIORITY
        && NVIC_GetEnableIRQ(HRTIM1_Master_IRQn) != 0U
        && NVIC_GetEnableIRQ(HRTIM1_TIMB_IRQn) != 0U
        && NVIC_GetEnableIRQ(DMA1_Channel1_IRQn) != 0U
        && NVIC_GetEnableIRQ(DMA1_Channel2_IRQn) != 0U
        && NVIC_GetEnableIRQ(USART1_IRQn) != 0U
        && NVIC_GetEnableIRQ(DMA1_Channel3_IRQn) != 0U
        && NVIC_GetEnableIRQ(FDCAN1_IT0_IRQn) != 0U;
}
