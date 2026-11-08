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
     * @details Kp 单位依次为 W/J、A/W、1/V、1/A；Ki 和 Kd 按原库逐次
     *          积分/差分，不自动乘除 dt。所有环每个控制周期计算一次。
     */
    /* 参照 Taurus Arm_PID_Init 的逐环调用格式；先清零再执行原库初始化。 */
    PID_Init(&g_power_ctrl.energy_pid, POWER_CTRL_POWER_MAX_W, POWER_CTRL_POWER_MAX_W, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, Integral_Limit);
    PID_Init(&g_power_ctrl.power_pid, POWER_CTRL_CAP_CURRENT_MAX_A,
             POWER_CTRL_CAP_CURRENT_MAX_A, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f,
             Integral_Limit | PID_EXTERNAL_ANTI_WINDUP | PID_OUTPUT_TRACKING);
    PID_Init(&g_power_ctrl.voltage_pid, BSP_HRTIM_GENERALIZED_DUTY_MAX,
             BSP_HRTIM_GENERALIZED_DUTY_MAX, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f,
             Integral_Limit | PID_EXTERNAL_ANTI_WINDUP | PID_OUTPUT_TRACKING);
    PID_Init(&g_power_ctrl.current_pid, BSP_HRTIM_GENERALIZED_DUTY_MAX,
             BSP_HRTIM_GENERALIZED_DUTY_MAX, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f,
             Integral_Limit | PID_EXTERNAL_ANTI_WINDUP | PID_OUTPUT_TRACKING);
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
 * @brief 先处理底盘过流和前馈可用性，再决定保护或连续正常恢复。
 * @pre 普通错误由 Master 发布；仅 ADC 上下文调用。
 * @note 高速电压判断只改变状态和恢复计数，普通电压错误位仍由 1 kHz 归因。
 *       无效电压不进行除法、不覆盖上次占空比；原始前馈低于调制下限也禁止开启。
 */
static void PowerCtrl_ModeSwitch(void)
{
    /* 过流仍先于前馈计算，命中本周期关断并锁存至 MCU 复位。 */
    if(fabsf(g_bsp_adc_feedback.adc_convert[I_CHASSIS]) > POWER_CTRL_CHASSIS_CURRENT_MAX_A)
    {
        g_power_ctrl.errors.bits.short_circuit = 1U;
        g_power_ctrl.state = PROTECT_MODE;
    }

    if(!(g_bsp_adc_feedback.adc_convert[V_CHASSIS] >= POWER_CTRL_INPUT_MIN_V
         && g_bsp_adc_feedback.adc_convert[V_CHASSIS] <= POWER_CTRL_OVERVOLTAGE_V
         && g_bsp_adc_feedback.adc_convert[V_CAP] > 0.0f
         && g_bsp_adc_feedback.adc_convert[V_CAP] <= POWER_CTRL_OVERVOLTAGE_V))
    {
        /* 直接保护，不以零前馈充当错误标志；NaN/无穷同样不满足有效区间。 */
        g_power_ctrl.state = PROTECT_MODE;
        g_power_ctrl.errclearcnt = 0U;
    }
    else
    {
        g_power_ctrl.generalized_duty = g_bsp_adc_feedback.adc_convert[V_CAP]
            / g_bsp_adc_feedback.adc_convert[V_CHASSIS];

        if(g_power_ctrl.errors.all != 0U
           || g_power_ctrl.generalized_duty < BSP_HRTIM_GENERALIZED_DUTY_MIN)
        {
            /* 比值可达性与普通电压错误分开，不新增错误位或预充状态机。 */
            g_power_ctrl.state = PROTECT_MODE;
            g_power_ctrl.errclearcnt = 0U;
        }
        else if(g_power_ctrl.state == PROTECT_MODE)
        {
            g_power_ctrl.errclearcnt++;
            if(g_power_ctrl.errclearcnt > POWER_CTRL_RESTART_DELAY_CYCLES)
            {
                g_power_ctrl.errclearcnt = 0U;
                /* 保护期间已持续预置；本周期先确认有效前馈，复位后单次开启。 */
                PowerCtrl_ResetLoops();
                g_power_ctrl.state = CTRL_MODE;
                BspHrtim_EnableOutput();
            }
        }
    }
}

/* ==================== 占空比计算与输出 ==================== */

/**
 * @brief 按已确定的状态关闭输出并预置，或执行闭环并更新调制。
 * @pre ModeSwitch 已处理本周期过流及前馈可用性；仅 ADC 上下文串行调用。
 * @note 前馈无效时保持旧比较记录但输出关闭，不通过底层最小限幅强行启动。
 */
static void PowerCtrl_DutyCalcu(void)
{
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
 * @brief 计算低压不对称目标限流、能量/功率环及电流与电压的占空比竞争。
 * @details 低于 5 V 充电上限 4 A、禁止主动放电；5～12 V 分别连续过渡到 13 A。
 *          B 端参考限值乘 Vcap/Vchassis 得到 A 端目标上下界，不引入效率参数。
 *          两个内环均输出相同前馈上的修正量，比较修正量等价于比较占空比。
 * @pre 前馈有效且不低于调制下限；generalized_duty 尚为本周期原始前馈。
 * @note 13 A 仅约束目标；电压环覆盖可改变实际电流，不能保证 B 端平均或峰值电流。
 *       OutputFilter 均关闭；先判断控制权和外环积分，再跟踪输家，最后覆盖前馈。
 */
static void PowerCtrl_CascadePidCalcu(void)
{
    /* 两个成员先作 B 端安培工作值，再原位换算为 A 端边界，避免另存同义副本。 */
    if(g_bsp_adc_feedback.adc_convert[V_CAP] < POWER_CTRL_CAP_LOW_VOLTAGE_V)
    {
        g_power_ctrl.current_upper_limit_a = POWER_CTRL_CAP_LOW_CHARGE_CURRENT_A;
        g_power_ctrl.current_lower_limit_a = 0.0f;
    }
    else if(g_bsp_adc_feedback.adc_convert[V_CAP] < POWER_CTRL_CAP_NORMAL_VOLTAGE_V)
    {
        g_power_ctrl.current_upper_limit_a = POWER_CTRL_CAP_LOW_CHARGE_CURRENT_A
            + (POWER_CTRL_CAP_CURRENT_MAX_A - POWER_CTRL_CAP_LOW_CHARGE_CURRENT_A)
                * (g_bsp_adc_feedback.adc_convert[V_CAP] - POWER_CTRL_CAP_LOW_VOLTAGE_V)
                / (POWER_CTRL_CAP_NORMAL_VOLTAGE_V - POWER_CTRL_CAP_LOW_VOLTAGE_V);
        g_power_ctrl.current_lower_limit_a = -POWER_CTRL_CAP_CURRENT_MAX_A
            * (g_bsp_adc_feedback.adc_convert[V_CAP] - POWER_CTRL_CAP_LOW_VOLTAGE_V)
            / (POWER_CTRL_CAP_NORMAL_VOLTAGE_V - POWER_CTRL_CAP_LOW_VOLTAGE_V);
    }
    else
    {
        g_power_ctrl.current_upper_limit_a = POWER_CTRL_CAP_CURRENT_MAX_A;
        g_power_ctrl.current_lower_limit_a = -POWER_CTRL_CAP_CURRENT_MAX_A;
    }
    g_power_ctrl.current_upper_limit_a *= g_power_ctrl.generalized_duty;
    g_power_ctrl.current_lower_limit_a *= g_power_ctrl.generalized_duty;
    g_power_ctrl.power_feedback_w = g_bsp_adc_feedback.adc_convert[V_CHASSIS]
        * g_bsp_adc_feedback.adc_convert[I_CHASSIS];

    PID_Calculate(&g_power_ctrl.energy_pid, power_heat_data.buffer_energy,
                  POWER_CTRL_ENERGY_TARGET_J);
    g_power_ctrl.power_target_w = g_power_ctrl.energy_pid.Output;

    /* 后续还要按控制权撤销同向积分，快照不能被本次 PID 状态覆盖。
     * 当前曲线正上界始终不小于负下界绝对值，故用正上界设置对称库内部限幅。
     * 旧积分先按新范围限制快照，防止撤销本次积分时恢复范围外的旧积分。 */
    float power_iout_before = g_power_ctrl.power_pid.Iout;
    g_power_ctrl.power_pid.MaxOut = g_power_ctrl.current_upper_limit_a;
    g_power_ctrl.power_pid.IntegralLimit = g_power_ctrl.current_upper_limit_a;
    OUTPUT_LIMIT(power_iout_before, g_power_ctrl.power_pid.IntegralLimit,
                 -g_power_ctrl.power_pid.IntegralLimit);
    /* 零误差死区只保留积分，须在计算前同步新范围，避免旧积分绕过动态限幅。 */
    g_power_ctrl.power_pid.Iout = power_iout_before;
    PID_CalculateConstrained(&g_power_ctrl.power_pid,
        g_power_ctrl.power_feedback_w, g_power_ctrl.power_target_w,
        g_power_ctrl.current_lower_limit_a, g_power_ctrl.current_upper_limit_a);

    PID_CalculateConstrained(&g_power_ctrl.current_pid,
        g_bsp_adc_feedback.adc_convert[I_CHASSIS], g_power_ctrl.power_pid.Output,
        BSP_HRTIM_GENERALIZED_DUTY_MIN - g_power_ctrl.generalized_duty,
        BSP_HRTIM_GENERALIZED_DUTY_MAX - g_power_ctrl.generalized_duty);
    /* 未参与竞争时也先更新当前 P/D 和误差历史，再跟踪实际输出。 */
    PID_CalculateConstrained(&g_power_ctrl.voltage_pid,
        g_bsp_adc_feedback.adc_convert[V_CAP], POWER_CTRL_CAP_TARGET_V,
        BSP_HRTIM_GENERALIZED_DUTY_MIN - g_power_ctrl.generalized_duty,
        BSP_HRTIM_GENERALIZED_DUTY_MAX - g_power_ctrl.generalized_duty);

    if(g_bsp_adc_feedback.adc_convert[V_CAP] > POWER_CTRL_VOLTAGE_COMPETE_V
       && g_power_ctrl.voltage_pid.Output < g_power_ctrl.current_pid.Output)
    {
        if(g_power_ctrl.power_pid.Iout > power_iout_before)
            g_power_ctrl.power_pid.Iout = power_iout_before;
        PID_TrackOutput(&g_power_ctrl.current_pid, g_power_ctrl.voltage_pid.Output);
    }
    else
    {
        PID_TrackOutput(&g_power_ctrl.voltage_pid, g_power_ctrl.current_pid.Output);
    }

    /* 电流 Output 此时记录实际采用的修正，内环全局饱和时撤销功率环同向积分。 */
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
    g_power_ctrl.current_upper_limit_a = 0.0f;
    g_power_ctrl.current_lower_limit_a = 0.0f;
    /* generalized_duty 是本周期已检查的前馈，进入 CTRL 时仍需叠加 PID 修正。 */
}
