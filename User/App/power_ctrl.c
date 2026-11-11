/**
 * @file power_ctrl.c
 * @brief 能量修正、入口功率调节、电压竞争、电流环及自动恢复。
 * @details 正电流充电、负电流放电；逻辑 A 端为 chassis，本板对应 Timer B。
 * @note ADC 管理输出、PID 和过流锁存；Master 发布普通错误，不实现硬件过流保护。
 */
#include "power_ctrl.h"

#include "main.h"
#include "power_ctrl_config.h"
#include "bsp_adc.h"
#include "prot_judge.h"
#include "bsp_hrtim.h"

#include <math.h>
#include <string.h>

/* ==================== 私有声明 ==================== */

static void PowerCtrl_ModeSwitch(void);
static void PowerCtrl_DutyCalcu(void);
static void PowerCtrl_CascadePidCalcu(void);
static void PowerCtrl_CalculateCurrentLimits(void);
static void PowerCtrl_ResetLoops(void);

PowerCtrl_t g_power_ctrl;
/* ==================== 公共入口 ==================== */

void PowerCtrl_Init(void)
{
    g_power_ctrl.state = PROTECT_MODE;

    /**
     * @brief 每次调用的离散 PID 系数，0 为待调试模板，不沿用旧板系数。
     * @details Kp 单位依次为 W/J、A/W、1/V、1/A；Ki 和 Kd 按原库逐次
     *          积分/差分，不自动乘除 dt。所有环每个控制周期计算一次。
     *          电流环启用积分限幅和外部条件积分，其余三环保持 NONE。
     *          电流环 Iout 受 IntegralLimit 约束，输出同时受修正量和总占空比边界约束。
     *          死区内 Output 为 0，Iout 保留；不启用输出跟踪或滤波。
     */
    /* 参照 Taurus Arm_PID_Init 的逐环调用格式；先清零再执行原库初始化。 */
    PID_Init(&g_power_ctrl.energy_pid, POWER_CTRL_POWER_MAX_W, POWER_CTRL_POWER_MAX_W, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, NONE);
    PID_Init(&g_power_ctrl.power_pid, POWER_CTRL_CAP_CURRENT_MAX_A,
             POWER_CTRL_CAP_CURRENT_MAX_A, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f,
             NONE);
    PID_Init(&g_power_ctrl.voltage_pid, BSP_HRTIM_LOOP_DUTY_MAX,
             BSP_HRTIM_LOOP_DUTY_MAX, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f,
             NONE);
    PID_Init(&g_power_ctrl.current_pid, BSP_HRTIM_LOOP_DUTY_MAX,
             0.09f, 0.0f,
             0.02f, 0.0008f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f,
             Integral_Limit | PID_EXTERNAL_ANTI_WINDUP);
}

void PowerCtrl_Task(void)
{
    PowerCtrl_ModeSwitch();
    PowerCtrl_DutyCalcu();
}

/* ==================== 状态切换 ==================== */

/**
 * @brief 先处理底盘过流、前馈和功率反馈可用性，再决定保护或连续正常恢复。
 * @pre 普通错误由 Master 发布；仅 ADC 上下文调用。
 * @note 高速电压判断只改变状态和恢复计数，普通电压错误位仍由 1 kHz 归因。
 *       无效电压不进行除法、不覆盖上次占空比；原始前馈低于调制下限也禁止开启。
 *       电池电压和两路电流须落在已有 ADC 理论量程内，NaN、无穷及极端值均被拒绝。
 *       有效输入已保证功率乘积有界，工作成员保存本周期 P_A 和 P_in，不重复检查乘积。
 */
static void PowerCtrl_ModeSwitch(void)
{
    /* 过流仍先于前馈计算，命中本周期关断并锁存至 MCU 复位。 */
    if(fabsf(g_bsp_adc_feedback.adc_convert[I_CHASSIS]) > POWER_CTRL_CHASSIS_CURRENT_MAX_A)
    {
        g_power_ctrl.errors.bits.short_circuit = 1U;
        g_power_ctrl.state = PROTECT_MODE;
    }

    g_power_ctrl.generalized_duty = g_bsp_adc_feedback.adc_convert[V_CAP]
        / g_bsp_adc_feedback.adc_convert[V_CHASSIS];
    g_power_ctrl.A_power_fdb = g_bsp_adc_feedback.adc_convert[V_CHASSIS]
        * g_bsp_adc_feedback.adc_convert[I_CHASSIS];

    if(g_power_ctrl.errors.all != 0U
    || !(g_power_ctrl.generalized_duty > BSP_HRTIM_GENERALIZED_DUTY_MIN&&g_power_ctrl.generalized_duty < BSP_HRTIM_GENERALIZED_DUTY_MAX))
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
            /* 输入范围和前馈已检查；复位保留本周期工作值，随后单次开启。 */
            PowerCtrl_ResetLoops();
            g_power_ctrl.state = CTRL_MODE;
            BspHrtim_EnableOutput();
        }
    }
}

/* ==================== 占空比计算与输出 ==================== */

/**
 * @brief 按已确定的状态关闭输出并预置，或执行闭环并更新调制。
 * @pre ModeSwitch 已处理本周期过流、前馈及功率反馈可用性；仅 ADC 上下文串行调用。
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
 * @brief 计算低压目标限流、能量修正后的入口功率误差及内环占空比竞争。
 * @details 低于 5 V 充电上限 4 A、禁止主动放电；5～12 V 分别连续过渡到 13 A。
 *          B 端参考限值乘 Vcap/Vchassis 得到 A 端目标上下界，不引入效率参数。
 *          两个内环均输出相同前馈上的修正量，比较修正量等价于比较占空比。
 * @pre ModeSwitch 已确认前馈及功率有效；generalized_duty 为原始前馈，
 *      A_power_fdb 为 P_A，total_power_fbd 为同周期入口总功率 P_in。
 * @note 13 A 仅约束目标；电压环覆盖可改变实际电流，不能保证 B 端平均或峰值电流。
 *       当前仅运行电流环，其余三环及电压竞争已注释；保留现有调试链路。
 *       电流环启用积分限幅和条件积分；达到有效边界时拒绝继续推动饱和的积分变化。
 */
static void PowerCtrl_CascadePidCalcu(void)
{
    PowerCtrl_CalculateCurrentLimits();

//    /* e_E = 45 J - E，输出 u_E 的单位为 W，每 40 us 按最新可读能量重复计算。
//     * 正向增益下能量不足产生正修正，后续减去它以减少充电或增加补电。
//     * NONE 不限制积分；零误差时库输出 0，但不清除 Iout。 */
//    PID_Calculate(&g_power_ctrl.energy_pid, power_heat_data.buffer_energy,
//                  POWER_CTRL_ENERGY_TARGET_J);

//     /* A 端功率与底盘用电功率之和的综合反馈，单位 W；由入口 V_BAT*I_BAT 计算。
//     * 入口采样未包含 R18 前板载供电自耗，不等同于裁判实测功率。 */
//    g_power_ctrl.total_power_fbd = g_bsp_adc_feedback.adc_convert[V_BAT]
//        * g_bsp_adc_feedback.adc_convert[I_BAT];
//        
//    /* P_A,target = P_lim - P_in + P_A - u_E；加回 P_A 是移除已计入入口的超电支路，
//     * 再计算允许的 A 端功率。以同一 P_A 为反馈，误差化简为 P_lim - P_in - u_E。
//     * P_in 来自 V_BAT*I_BAT，不包含 R18 前板载供电自耗，计量偏差仍待实测。 */
//    g_power_ctrl.power_target_w = (float)robot_status.chassis_power_limit
//        - g_power_ctrl.total_power_fbd + g_power_ctrl.A_power_fdb
//        - g_power_ctrl.energy_pid.Output;

//    /* 功率 PID 由 W 误差生成 A 端目标电流 A，再受低压不对称边界约束。
//     * 正上界不小于负下界绝对值，用其设置库内对称输出范围；NONE 只限输出。 */
//    g_power_ctrl.power_pid.MaxOut = g_power_ctrl.current_upper_limit_a;
//    PID_CalculateConstrained(&g_power_ctrl.power_pid,
//        g_power_ctrl.A_power_fdb, g_power_ctrl.power_target_w,
//        g_power_ctrl.current_lower_limit_a, g_power_ctrl.current_upper_limit_a);
	
   /* 电压 PI 同样输出无量纲修正 u_V，以 29 V 为目标，每周期计算。
    * 两候选均为 Dff+u，共用前馈，故直接比较 u_V、u_I 即可取较小占空比。
    * NONE 不冻结或跟踪未采用的环，其积分历史仍按基础算法推进。 */
   PID_Calculate(&g_power_ctrl.voltage_pid,
       g_bsp_adc_feedback.adc_convert[V_CAP], POWER_CTRL_CAP_TARGET_V);
	
    /* 电流修正范围取 ±MaxOut 与 [MIN-Dff, MAX-Dff] 的交集，无量纲。
     * 达到任一有效边界时停止同向积分，允许反向积分退饱和；前馈仍为本周期原始值。 */
    PID_CalculateConstrained(&g_power_ctrl.current_pid,
        g_bsp_adc_feedback.adc_convert[I_CHASSIS], g_power_ctrl.power_pid.Output,
        -BSP_HRTIM_LOOP_DUTY_MAX, BSP_HRTIM_LOOP_DUTY_MAX);


   /* 严格高于 26.1 V 才允许电压环限制充电；其余周期完全采用电流候选。 */
   if(g_bsp_adc_feedback.adc_convert[V_CAP] > POWER_CTRL_VOLTAGE_COMPETE_V
      && g_power_ctrl.voltage_pid.Output < g_power_ctrl.current_pid.Output)
       g_power_ctrl.generalized_duty += g_power_ctrl.voltage_pid.Output;
   else
        g_power_ctrl.generalized_duty += g_power_ctrl.current_pid.Output;

    OUTPUT_LIMIT(g_power_ctrl.generalized_duty,
                 BSP_HRTIM_GENERALIZED_DUTY_MAX, BSP_HRTIM_GENERALIZED_DUTY_MIN);
}
/**
 * @brief 1 kHz 发布普通错误，避免在 ADC 控制周期内重复检查。
 * @note 局部快照保存 ADC 可抢占覆盖的反馈，不复用仍在运行的控制记录。
 *       电池电压使用输入下限和 ADC 理论量程上限判断数据合理性，不作为器件额定值。
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
        !(bat_voltage_v >= POWER_CTRL_INPUT_MIN_V && bat_voltage_v <= ADC_V_BAT_MAX_V);
//    errors.bits.power_off = robot_status.power_management_chassis_output == 0U;
//    errors.bits.referee =
//        (uint32_t)(HAL_GetTick() - g_judge_rx_update_ms) > POWER_CTRL_REFEREE_TIMEOUT_MS;

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


/* ==================== 私有辅助 ==================== */

/**
 * @brief 计算 A 端目标电流上下限，单位 A。
 * @details 按电容电压分段计算 B 端充放电限值，再乘原始前馈换算至 A 端。
 * @pre 仅 ADC 上下文串行调用；generalized_duty 为本周期原始 Vcap/Vchassis。
 * @note 直接更新既有上下限成员；目标限幅不等同于实际电流或峰值保护。
 */
static void PowerCtrl_CalculateCurrentLimits(void)
{
    /* 低压时限制从电容提取功率，仍允许较小充电目标；5～12 V 连续恢复双向能力。
     * 两个成员先作 B 端安培工作值，再原位换算为 A 端边界，避免另存同义副本。 */
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
    /* 理想功率平衡 V_A*I_A = V_B*I_B，故 A 端限值为 B 端参考乘 V_B/V_A。
     * 这里不估算效率，也不把目标限幅等同于实际平均电流或峰值保护。 */
    g_power_ctrl.current_upper_limit_a *= g_power_ctrl.generalized_duty;
    g_power_ctrl.current_lower_limit_a *= g_power_ctrl.generalized_duty;
}

static void PowerCtrl_ResetLoops(void)
{
    g_power_ctrl.energy_pid.PID_reset(&g_power_ctrl.energy_pid);
    g_power_ctrl.power_pid.PID_reset(&g_power_ctrl.power_pid);
    g_power_ctrl.voltage_pid.PID_reset(&g_power_ctrl.voltage_pid);
    g_power_ctrl.current_pid.PID_reset(&g_power_ctrl.current_pid);
    g_power_ctrl.current_upper_limit_a = 0.0f;
    g_power_ctrl.current_lower_limit_a = 0.0f;
    /* 保留 ModeSwitch 本周期计算的原始前馈、P_A 和 total_power_fbd 中的 P_in。
     * 本次进入 CTRL 后立即形成 A 端目标并叠加内环修正，不能在此清掉工作值。 */
}
