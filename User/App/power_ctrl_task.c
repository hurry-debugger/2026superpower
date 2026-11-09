/**
 * @file power_ctrl_task.c
 * @brief ADC1 DMA 25 kHz 采样/过流/控制，Master 1 kHz 普通保护及 CAN 调度。
 */
#include "power_ctrl_task.h"

#include "boardinfo.h"

#include "adc.h"
#include "bsp_adc.h"
#include "bsp_hrtim.h"
#include "bsp_usart.h"
#include "hrtim.h"
#include "power_ctrl.h"
#include "prot_supercap.h"
#include "bsp_dwt.h"

/**
 * @brief 先保持输出关闭并准备接收，再启动产生采样和保护事件的计数器。
 * @note 初始化不直接开启闭环输出；ADC 状态机满足保护及恢复条件后负责开启。
 */
void SuperPower_Init(void)
{
    BspHrtim_DisableOutput();

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    BoardInfo_Init();

    /* UID 匹配已完成；只在启动时由掩码生成校准汇总，不重复扫描详细错误。 */
    g_power_ctrl.errors.bits.calibration =
        g_board_info.valid_channel_mask != ADC_CAL_ALL_CHANNELS_MASK;
    
    Bsp_DWT_Init();

    /* 计数器尚未启动，先准备 ADC DMA 和通信，避免首个控制事件使用未初始化接口。 */
    BspAdcFeedback_Init();
    USER_UART_Init();
    SupercapCan_Init();
    
    PowerCtrl_Init();

    HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
}

/**
 * @brief ADC1 全满事件选择闭环采样控制或开环固定 PWM；ADC2 不驱动控制。
 * @param[in] hadc 完成转换的 ADC 句柄。
 * @note ADC1 DMA 为优先级 0；两路循环 DMA 在处理期间继续采样。
 *       闭环先完成反馈换算，再依次检查保护和计算四环，保证使用本次发布的 ADC 值。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc == &hadc1)
    {
        // 测量25 kHz 采样周期
		Bsp_DWT_Stop(0);
		Bsp_DWT_Start(0);
		
		Bsp_DWT_Start(1);
#if defined(POWER_CTRL_CLOSED_LOOP)
        BspAdcFeedback_Process();
        PowerCtrl_Task();
#else
        /* 开环沿用固定比较值，每次回调都调用开启，不执行采样、闭环或错误关断。 */
        BspHrtim_EnableOutput();
#endif
		
		Bsp_DWT_Stop(1);
    }
}

/**
 * @brief Master 1 kHz 先检查普通故障，再快照并发送经典 CAN。
 * @param[in] hhrtim 产生事件的 HRTIM 句柄。
 * @param[in] timer_index 产生重复事件的计数器索引。
 * @note Master 为优先级 1，允许 ADC1 DMA 抢占；只发布错误字，不开启 PWM 或复位 PID。
 *       闭环 ADC 下一周期据错误字保护；开环不执行控制状态机，错误字仅供状态反馈。
 */
void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t timer_index)
{
    if(hhrtim == &hhrtim1 && timer_index == HRTIM_TIMERINDEX_MASTER)
    {
        PowerCtrl_UpdateErrors();
        SupercapCan_SendStatus();
    }
}
