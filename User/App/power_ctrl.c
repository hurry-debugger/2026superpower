/**
 * @file power_ctrl.c
 * @brief 能量环、A 端功率环、电压竞争、电流环及自动恢复。
 * @details 正电流充电、负电流放电；逻辑 A 端为 chassis，本板对应 Timer B。
 * @note ADC 管理输出、PID 和过流锁存；Master 发布普通错误，不实现硬件过流保护。
 */
#include "power_ctrl.h"

#include "main.h"
#include "power_ctrl_config.h"
#include "bsp_adc.h"
#include "prot_judge.h"
#include "bsp_hrtim.h"

#include <float.h>
#include <math.h>
#include <string.h>

/* ==================== 私有声明 ==================== */

static void PowerCtrl_ModeSwitch(void);
static void PowerCtrl_DutyCalcu(void);
static void PowerCtrl_CascadePidCalcu(void);
static void PowerCtrl_ResetLoops(void);

PowerCtrl_t g_power_ctrl;
/* ==================== 公共入口 ==================== */

void PowerCtrl_Init(void)
{
    BspHrtim_Init();
    memset(&g_power_ctrl, 0, sizeof(g_power_ctrl));
    g_power_ctrl.state = PROTECT_MODE;

    /**
     * @brief 每次调用的离散 PID 系数，0 为待调试模板，不沿用旧板系数。
     * @details Kp 单位依次为 W/J、A/W、A/V、1/A；Ki 和 Kd 按原库逐次
     *          积分/差分，不自动乘除 dt。所有环每个控制周期计算一次。
     */
    /* 参照 Taurus Arm_PID_Init 的逐环调用格式；先清零再执行原库初始化。 */
    PID_Init(&g_power_ctrl.energy_pid, POWER_CTRL_POWER_MAX_W, POWER_CTRL_POWER_MAX_W, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, Integral_Limit);
    PID_Init(&g_power_ctrl.power_pid, POWER_CTRL_CAP_CURRENT_MAX_A,
             POWER_CTRL_CAP_CURRENT_MAX_A, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, Integral_Limit | PID_EXTERNAL_ANTI_WINDUP);
    PID_Init(&g_power_ctrl.voltage_pid, POWER_CTRL_CAP_CURRENT_MAX_A,
             POWER_CTRL_CAP_CURRENT_MAX_A, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, Integral_Limit);
    PID_Init(&g_power_ctrl.current_pid, BSP_HRTIM_GENERALIZED_DUTY_MAX,
             BSP_HRTIM_GENERALIZED_DUTY_MAX, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, Integral_Limit | PID_EXTERNAL_ANTI_WINDUP);
}

void PowerCtrl_Task(void)
{
    PowerCtrl_ModeSwitch();
    PowerCtrl_DutyCalcu();
}

/**
 * @brief 1 kHz 发布普通错误，避免在 ADC 控制周期内重复检查。
 * @note 局部快照保存 ADC 可抢占覆盖的反馈，不复用仍在运行的控制记录。
 *       FLT_MAX 仅拒绝非有限数值，不是电池物理电压上限。
 */
void PowerCtrl_UpdateErrors(void)
{
    float cap_voltage_v;
    float chassis_voltage_v;
    float bat_voltage_v;
    PowerCtrlError_t errors;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();
    cap_voltage_v = g_bsp_adc_feedback.adc_convert[V_CAP];
    chassis_voltage_v = g_bsp_adc_feedback.adc_convert[V_CHASSIS];
    bat_voltage_v = g_bsp_adc_feedback.adc_convert[V_BAT];
    errors = g_power_ctrl.errors;
    __DMB();
    __set_PRIMASK(primask);

    errors.bits.cap_voltage =
        !(cap_voltage_v > 0.0f && cap_voltage_v <= POWER_CTRL_OVERVOLTAGE_V);
    errors.bits.bat_voltage =
        !(bat_voltage_v >= POWER_CTRL_INPUT_MIN_V && bat_voltage_v <= FLT_MAX);
    errors.bits.power_off = robot_status.power_management_chassis_output == 0U;
    errors.bits.referee =
        (uint32_t)(HAL_GetTick() - g_judge_rx_update_ms) > POWER_CTRL_REFEREE_TIMEOUT_MS;

    if(!(chassis_voltage_v >= POWER_CTRL_INPUT_MIN_V
         && chassis_voltage_v <= POWER_CTRL_OVERVOLTAGE_V))
    {
        /* 硬过压、输入丢失及无效电压本次即保护，不等待累计。
         * 数值已回落到最低档以下时清零历史，下电也不能遗留旧累计。 */
        if(chassis_voltage_v <= POWER_CTRL_OVERVOLTAGE_LEVEL_1_V)
            g_power_ctrl.chassis_overvoltage_cnt = 0U;
        errors.bits.chassis_voltage = 1U;
    }
    else
    {
        /* 从高到低只选择一步；跨档保留累计，不另存档位或步长。 */
        if(chassis_voltage_v > POWER_CTRL_OVERVOLTAGE_LEVEL_4_V)
            g_power_ctrl.chassis_overvoltage_cnt += POWER_CTRL_OVERVOLTAGE_STEP_4;
        else if(chassis_voltage_v > POWER_CTRL_OVERVOLTAGE_LEVEL_3_V)
            g_power_ctrl.chassis_overvoltage_cnt += POWER_CTRL_OVERVOLTAGE_STEP_3;
        else if(chassis_voltage_v > POWER_CTRL_OVERVOLTAGE_LEVEL_2_V)
            g_power_ctrl.chassis_overvoltage_cnt += POWER_CTRL_OVERVOLTAGE_STEP_2;
        else if(chassis_voltage_v > POWER_CTRL_OVERVOLTAGE_LEVEL_1_V)
            g_power_ctrl.chassis_overvoltage_cnt += POWER_CTRL_OVERVOLTAGE_STEP_1;
        else
            g_power_ctrl.chassis_overvoltage_cnt = 0U;

        if(g_power_ctrl.chassis_overvoltage_cnt >= POWER_CTRL_OVERVOLTAGE_ACCUM_LIMIT)
            g_power_ctrl.chassis_overvoltage_cnt = POWER_CTRL_OVERVOLTAGE_ACCUM_LIMIT;
        errors.bits.chassis_voltage =
            g_power_ctrl.chassis_overvoltage_cnt >= POWER_CTRL_OVERVOLTAGE_ACCUM_LIMIT;
    }

    /* 位域写入为读改写，必须保留 ADC 新置位的短路锁存后再原子发布。
     * 临界区只做复制/发布，判断、PID 和 PWM HAL 调用均在区外。 */
    primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    errors.bits.short_circuit = g_power_ctrl.errors.bits.short_circuit;
    g_power_ctrl.errors = errors;
    __DMB();
    __set_PRIMASK(primask);
}

/* ==================== 状态切换 ==================== */

/**
 * @brief 下电或故障时保护；满足全部条件后累计恢复周期并单次开启。
 * @pre 普通错误由 Master 发布；短路已在本次 ADC 控制入口检查。
 * @note 计数仅在 PROTECT 中递增；保护期间持续预置平衡点，CTRL 不重复复位或开启。
 */
static void PowerCtrl_ModeSwitch(void)
{
    /* 用户指定仅以底盘电流判短路/过流，本周期保护分支直接关断。 */
    if(fabsf(g_bsp_adc_feedback.adc_convert[I_CHASSIS]) > POWER_CTRL_CHASSIS_CURRENT_MAX_A)
    {
        g_power_ctrl.errors.bits.short_circuit = 1U;
        g_power_ctrl.state = PROTECT_MODE;
    }

    if(g_power_ctrl.errors.all != 0U)
    {
        g_power_ctrl.errclearcnt = 0U;
        g_power_ctrl.state = PROTECT_MODE;
    }
    else if(g_power_ctrl.state == PROTECT_MODE)
    {
        g_power_ctrl.errclearcnt++;
        if(g_power_ctrl.errclearcnt > POWER_CTRL_RESTART_DELAY_CYCLES)
        {
            g_power_ctrl.errclearcnt = 0U;
            /* 保护期间已持续装载平衡点；复位保留本周期前馈，随后立即执行控制。 */
            PowerCtrl_ResetLoops();
            g_power_ctrl.state = CTRL_MODE;
            BspHrtim_EnableOutput();
        }
    }
}

/* ==================== 占空比计算与输出 ==================== */

/**
 * @brief 保护期间持续预置前馈，恢复后执行闭环控制。
 * @pre 已完成错误检查；仅由固定周期 Task 串行调用，计数器及预装载持续运行。
 */
static void PowerCtrl_DutyCalcu(void)
{
    if(g_bsp_adc_feedback.adc_convert[V_CHASSIS] >= POWER_CTRL_INPUT_MIN_V
       && g_bsp_adc_feedback.adc_convert[V_CHASSIS] <= POWER_CTRL_OVERVOLTAGE_V
       && g_bsp_adc_feedback.adc_convert[V_CAP] > 0.0f
       && g_bsp_adc_feedback.adc_convert[V_CAP] <= POWER_CTRL_OVERVOLTAGE_V)
    {
        g_power_ctrl.generalized_duty = g_bsp_adc_feedback.adc_convert[V_CAP]
            / g_bsp_adc_feedback.adc_convert[V_CHASSIS];
    }
    else
    {
        /* 输入电压不可用时记录零前馈；底层统一限幅，输出保护由错误位决定。 */
        g_power_ctrl.state = PROTECT_MODE;
    }

    // clang-format off
    switch(g_power_ctrl.state)
    {
        case PROTECT_MODE:
        {
            BspHrtim_DisableOutput();
            BspHrtim_SetGeneralizedDuty(g_power_ctrl.generalized_duty);
        }    break;

        case CTRL_MODE:
        {
            PowerCtrl_CascadePidCalcu();
            BspHrtim_SetGeneralizedDuty(g_power_ctrl.generalized_duty);
        }    break;

        default:
        {
            BspHrtim_DisableOutput();
            g_power_ctrl.state = PROTECT_MODE;
            g_power_ctrl.errclearcnt = 0U;
        }    break;
    }
    // clang-format on
}

/**
 * @brief 正常控制的能量、功率、电压竞争和电流串级计算，不操作硬件输出。
 * @details 能量环、电压环只做普通 PID 计算与输出限幅。
 *          先得到电压环允许的正向电流，再由 PID 库约束功率环和电流环。
 *          电流环 Output 为广义占空比修正，叠加 Vcap/Vchassis 前馈后限幅。
 * @pre 反馈由 ADC 本周期更新，普通电压错误由 1 kHz 发布；generalized_duty 为本周期前馈。
 */
static void PowerCtrl_CascadePidCalcu(void)
{
    g_power_ctrl.current_limit_a = POWER_CTRL_CAP_CURRENT_MAX_A * g_power_ctrl.generalized_duty;
    g_power_ctrl.power_feedback_w = g_bsp_adc_feedback.adc_convert[V_CHASSIS]
        * g_bsp_adc_feedback.adc_convert[I_CHASSIS];

    PID_Calculate(&g_power_ctrl.energy_pid, power_heat_data.buffer_energy,
                  POWER_CTRL_ENERGY_TARGET_J);

    g_power_ctrl.power_target_w = g_power_ctrl.energy_pid.Output;

    if(g_bsp_adc_feedback.adc_convert[V_CAP] > POWER_CTRL_VOLTAGE_COMPETE_V)
    {
        g_power_ctrl.voltage_pid.MaxOut = g_power_ctrl.current_limit_a;
        g_power_ctrl.voltage_pid.IntegralLimit = g_power_ctrl.current_limit_a;
        PID_Calculate(&g_power_ctrl.voltage_pid,
            g_bsp_adc_feedback.adc_convert[V_CAP], POWER_CTRL_CAP_TARGET_V);
        OUTPUT_LIMIT(g_power_ctrl.voltage_pid.Output, g_power_ctrl.current_limit_a, 0.0f);
        if(g_bsp_adc_feedback.adc_convert[V_CAP] >= POWER_CTRL_CAP_TARGET_V
           && g_power_ctrl.voltage_pid.Output >= 0.0f
           && g_power_ctrl.voltage_pid.Output <= g_power_ctrl.current_limit_a)
            g_power_ctrl.voltage_pid.Output = 0.0f;
    }
    else
    {
        g_power_ctrl.voltage_pid.PID_reset(&g_power_ctrl.voltage_pid);
        g_power_ctrl.voltage_pid.Output = g_power_ctrl.current_limit_a;
    }

    /* 电流环饱和后仍需读取功率环计算前积分，不能用已更新的 Iout 替代此快照。 */
    float power_iout_before = g_power_ctrl.power_pid.Iout;
    g_power_ctrl.power_pid.MaxOut = g_power_ctrl.current_limit_a;
    g_power_ctrl.power_pid.IntegralLimit = g_power_ctrl.current_limit_a;
    PID_CalculateConstrained(&g_power_ctrl.power_pid,
        g_power_ctrl.power_feedback_w, g_power_ctrl.power_target_w,
        -g_power_ctrl.current_limit_a, g_power_ctrl.voltage_pid.Output);

    PID_CalculateConstrained(&g_power_ctrl.current_pid,
        g_bsp_adc_feedback.adc_convert[I_CHASSIS], g_power_ctrl.power_pid.Output,
        BSP_HRTIM_GENERALIZED_DUTY_MIN - g_power_ctrl.generalized_duty,
        BSP_HRTIM_GENERALIZED_DUTY_MAX - g_power_ctrl.generalized_duty);

    /* 内环已到占空比边界时，功率环停止积累同向请求。 */
    if((g_power_ctrl.current_pid.Output >= BSP_HRTIM_GENERALIZED_DUTY_MAX
            - g_power_ctrl.generalized_duty
        && g_power_ctrl.power_pid.Iout > power_iout_before)
       || (g_power_ctrl.current_pid.Output <= BSP_HRTIM_GENERALIZED_DUTY_MIN
            - g_power_ctrl.generalized_duty
           && g_power_ctrl.power_pid.Iout < power_iout_before))
        g_power_ctrl.power_pid.Iout = power_iout_before;
        
    g_power_ctrl.generalized_duty += g_power_ctrl.current_pid.Output;
    OUTPUT_LIMIT(g_power_ctrl.generalized_duty,
                 BSP_HRTIM_GENERALIZED_DUTY_MAX, BSP_HRTIM_GENERALIZED_DUTY_MIN);
}

/* ==================== 私有辅助 ==================== */

static void PowerCtrl_ResetLoops(void)
{
    g_power_ctrl.energy_pid.PID_reset(&g_power_ctrl.energy_pid);
    g_power_ctrl.power_pid.PID_reset(&g_power_ctrl.power_pid);
    g_power_ctrl.voltage_pid.PID_reset(&g_power_ctrl.voltage_pid);
    g_power_ctrl.current_pid.PID_reset(&g_power_ctrl.current_pid);
    g_power_ctrl.power_target_w = 0.0f;
    g_power_ctrl.current_limit_a = 0.0f;
    /* generalized_duty 是本周期已检查的前馈，进入 CTRL 时仍需叠加 PID 修正。 */
}
