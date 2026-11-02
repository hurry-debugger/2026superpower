/**
 * @file bsp_hrtim.c
 * @brief 本板广义占空比分配、中心对称比较值更新和四路功率输出启停。
 * @details 分配公式及迟滞初值参考港科 RM2024 PowerManager.cpp 的 updatePWM()；
 *          按本板原理图重新映射物理侧和上下管，不沿用参考工程的 Timer A/B 含义。
 * @note 参数和波形尚未上板验证；调用方须核对 CubeMX 更新配置；运行期仅 Master 持有输出和占空比。
 */

#include "bsp_hrtim.h"

#include "hrtim.h"

#include <stdbool.h>
#include <stdint.h>

/* ==================== 私有定义 ==================== */

/** @brief 本板四路功率输出，来源为当前 HRTIM1 引脚配置及原理图第 3 页。 */
#define BSP_HRTIM_POWER_OUTPUT_MASK \
    (HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2)

/** @brief 同时禁止 A/B 预装载到活动寄存器的传输；位定义来自 STM32G4 CMSIS。 */
#define BSP_HRTIM_UPDATE_DISABLE_MASK (HRTIM_CR1_TAUDIS | HRTIM_CR1_TBUDIS)

/** @brief 中间调制模式的迟滞状态；仅由 Init 和占空比更新入口修改。 */
static bool s_buck_boost_mode;

/* ==================== 公共实现 ==================== */

void BspHrtim_Init(void)
{
    BspHrtim_DisableOutput();
    s_buck_boost_mode = false;
}

void BspHrtim_SetGeneralizedDuty(float generalized_duty)
{
    float chassis_upper_duty;
    float cap_upper_duty;

    if(generalized_duty < BSP_HRTIM_GENERALIZED_DUTY_MIN)
        generalized_duty = BSP_HRTIM_GENERALIZED_DUTY_MIN;
    else if(generalized_duty > BSP_HRTIM_GENERALIZED_DUTY_MAX)
        generalized_duty = BSP_HRTIM_GENERALIZED_DUTY_MAX;

    if(s_buck_boost_mode)
    {
        if(generalized_duty < BSP_HRTIM_BUCK_BOOST_EXIT_MIN
           || generalized_duty > BSP_HRTIM_BUCK_BOOST_EXIT_MAX)
        {
            s_buck_boost_mode = false;
        }
    }
    else if(generalized_duty > BSP_HRTIM_BUCK_BOOST_ENTER_MIN
            && generalized_duty < BSP_HRTIM_BUCK_BOOST_ENTER_MAX)
    {
        s_buck_boost_mode = true;
    }

    /* 两种分配均保持理想上管占空比之比 D_chassis / D_cap = generalized_duty。 */
    if(s_buck_boost_mode)
    {
        chassis_upper_duty = BSP_HRTIM_BUCK_BOOST_SCALE * (generalized_duty + 1.0f);
        cap_upper_duty = BSP_HRTIM_BUCK_BOOST_SCALE * (1.0f / generalized_duty + 1.0f);
    }
    else if(generalized_duty < 1.0f)
    {
        chassis_upper_duty = BSP_HRTIM_BASE_UPPER_DUTY * generalized_duty;
        cap_upper_duty = BSP_HRTIM_BASE_UPPER_DUTY;
    }
    else
    {
        chassis_upper_duty = BSP_HRTIM_BASE_UPPER_DUTY;
        cap_upper_duty = BSP_HRTIM_BASE_UPPER_DUTY / generalized_duty;
    }

    const uint32_t cap_period_count = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR;
    const uint32_t chassis_period_count =
        hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].PERxR;

    /*
     * CH1 接下管，中心窗口宽度为 P * (1 - D_upper)，故 CMP1 = P * D_upper / 2。
     * 正值加 0.5 后截断实现四舍五入，CMP2 用整数相减保持关于周期中点对称。
     * 这里是理想几何关系，不将死区计数直接当作上管导通时间补偿。
     */
    const uint32_t cap_compare_start_count =
        (uint32_t)((float)cap_period_count * 0.5f * cap_upper_duty + 0.5f);
    const uint32_t cap_compare_end_count = cap_period_count - cap_compare_start_count;
    const uint32_t chassis_compare_start_count =
        (uint32_t)((float)chassis_period_count * 0.5f * chassis_upper_duty + 0.5f);
    const uint32_t chassis_compare_end_count = chassis_period_count - chassis_compare_start_count;

    const uint32_t saved_update_disable =
        hhrtim1.Instance->sCommonRegs.CR1 & BSP_HRTIM_UPDATE_DISABLE_MASK;

    /* 禁止硬件在四次写入之间装载半组参数，不改变调用方原有的更新禁止状态。 */
    SET_BIT(hhrtim1.Instance->sCommonRegs.CR1, BSP_HRTIM_UPDATE_DISABLE_MASK);
    hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = cap_compare_start_count;
    hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP2xR = cap_compare_end_count;
    hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = chassis_compare_start_count;
    hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP2xR = chassis_compare_end_count;
    MODIFY_REG(hhrtim1.Instance->sCommonRegs.CR1, BSP_HRTIM_UPDATE_DISABLE_MASK,
               saved_update_disable);
}

void BspHrtim_EnableOutput(void)
{
    if(HAL_HRTIM_WaveformOutputStart(&hhrtim1, BSP_HRTIM_POWER_OUTPUT_MASK) != HAL_OK)
    {
        Error_Handler();
    }
}

void BspHrtim_DisableOutput(void)
{
    if(HAL_HRTIM_WaveformOutputStop(&hhrtim1, BSP_HRTIM_POWER_OUTPUT_MASK) != HAL_OK)
    {
        Error_Handler();
    }
}
