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
     * @brief 系数按每次调用定义；能量、功率环位置式，电压、电流环增量式。
     * @details Kp 单位依次为 W/J、A/W、1/V、1/A；Ki、Kd 按逐次积分/差分定义，
     *          不自动乘除 dt。内环 Output 为完整广义占空比，初始化设置范围 [0.05,29/24]。
     *          四环 Improve 均为 NONE，不启用滤波、位置式条件积分或输出跟踪。
     *          内环 IntegralLimit 参数保留，增量式 Iout 为本次 Ki*Err，不使用该限值。
     */
    /* 实例在启动时由全局零初始化提供初值，再按原库格式逐环设置参数。 */
    PID_Init(&g_power_ctrl.energy_pid, POWER_CTRL_POWER_MAX_W,
             -POWER_CTRL_POWER_MAX_W, POWER_CTRL_POWER_MAX_W, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, NONE);
    PID_Init(&g_power_ctrl.power_pid, POWER_CTRL_CAP_CURRENT_MAX_A,
             -POWER_CTRL_CAP_CURRENT_MAX_A,
             POWER_CTRL_CAP_CURRENT_MAX_A, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f,
             NONE);
    PID_Init(&g_power_ctrl.voltage_pid, POWER_CTRL_OUTPUT_DUTY_MAX,
             BSP_HRTIM_GENERALIZED_DUTY_MIN,
             BSP_HRTIM_LOOP_DUTY_MAX, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f,
             NONE);
    PID_Init(&g_power_ctrl.current_pid, POWER_CTRL_OUTPUT_DUTY_MAX,
             BSP_HRTIM_GENERALIZED_DUTY_MIN,
             0.09f, 0.0f,
             0.02f, 0.0008f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f,
             NONE);
}

void PowerCtrl_Task(void)
{
    PowerCtrl_ModeSwitch();
    PowerCtrl_DutyCalcu();
}

/* ==================== 状态切换 ==================== */

/**
 * @brief 处理底盘过流、电容范围及原始前馈，再决定保护或连续正常恢复。
 * @pre 普通错误由 Master 发布；仅 ADC 上下文调用。
 * @note 按当前调试路径直接计算 Vcap/Vchassis，原始前馈仍要求处于 (0.05,10)。
 *       输出上限 29/24 不参与前馈可用性判断；本轮不增加历史文档中的输入预检。
 *       恢复时清环历史并以本周期前馈预置电流环，首个 CTRL 周期再累计一次增量。
 */
static void PowerCtrl_ModeSwitch(void)
{
    /* 过流仍先于前馈计算，命中本周期关断并锁存至 MCU 复位。 */
    if(fabsf(g_bsp_adc_feedback.adc_convert[I_CHASSIS]) > POWER_CTRL_CHASSIS_CURRENT_MAX_A)
        g_power_ctrl.errors.bits.short_circuit = 1U;
        
    g_power_ctrl.errors.bits.cap_voltage = !(g_bsp_adc_feedback.adc_convert[V_CAP] >= POWER_CTRL_CAP_MIN_VOLTAGE_V 
                                                && g_bsp_adc_feedback.adc_convert[V_CAP] <= POWER_CTRL_OVERVOLTAGE_V);

    g_power_ctrl.f_generalized_duty = g_bsp_adc_feedback.adc_convert[V_CAP]
        / g_bsp_adc_feedback.adc_convert[V_CHASSIS];
    g_power_ctrl.A_power_fdb = g_bsp_adc_feedback.adc_convert[V_CHASSIS]
        * g_bsp_adc_feedback.adc_convert[I_CHASSIS];

    if(g_power_ctrl.errors.all != 0U
       || !(g_power_ctrl.f_generalized_duty > BSP_HRTIM_GENERALIZED_DUTY_MIN
            && g_power_ctrl.f_generalized_duty < BSP_HRTIM_GENERALIZED_DUTY_MAX))
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
            /* 原始前馈已通过现有门槛；先预置电流环，实际输出在计算增量后限幅。 */
            PowerCtrl_ResetLoops();
            g_power_ctrl.current_pid.Output = g_power_ctrl.f_generalized_duty;
            g_power_ctrl.state = CTRL_MODE;
            BspHrtim_EnableOutput();
        }
    }
}

/* ==================== 占空比计算与输出 ==================== */

/**
 * @brief 保护时关闭输出并传递前馈预置，CTRL 时采用内环完整占空比更新调制。
 * @pre ModeSwitch 已执行本周期现有保护和前馈门控；仅 ADC 上下文串行调用。
 * @note 保护预置传递原始前馈，沿用底层 [0.05,10] 限幅；NaN 跳过寄存器写入。
 */
static void PowerCtrl_DutyCalcu(void)
{
    // clang-format off
    switch(g_power_ctrl.state)
    {
        case PROTECT_MODE:
        {
            BspHrtim_DisableOutput();
            g_power_ctrl.generalized_duty = g_power_ctrl.f_generalized_duty;
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
 * @brief 计算低压目标限流及两个增量式内环，再按资格和候选占空比竞争。
 * @details B 端目标限值乘原始 Vcap/Vchassis 得到 A 端边界，分段规则保持。
 *          两环每周期各计算一次，未采用的环也推进历史，Output 为累计完整占空比。
 * @pre ModeSwitch 已执行现有门控；f_generalized_duty 为本周期原始前馈，A_power_fdb 为 P_A。
 * @note 仅正目标电流允许电压竞争；能量环和功率环保持注释，不启用输出跟踪。
 *       刚接管的环改用前馈加本次 ITerm 并限幅，不清误差历史、不重复计算或竞争。
 *       13 A 仅约束目标；电压覆盖期间不能保证 B 端实际平均或峰值电流。
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
	
    /**
     * @brief 增量式两内环各推进一次历史；电压目标为最大电压的 95%（当前 27.55 V）。
     * @note PID 内部按初始化的 MinOut/MaxOut 限制候选并对齐 Last_Output，再进行本周期竞争。
     *       不再使用前馈上的 ±0.2 修正边界，前馈仅在进入 CTRL 或控制权切换时预置。
     */
    PID_Calculate_Increase(&g_power_ctrl.voltage_pid,
        g_bsp_adc_feedback.adc_convert[V_CAP], POWER_CTRL_CAP_TARGET_V);

    PID_Calculate_Increase(&g_power_ctrl.current_pid,
        g_bsp_adc_feedback.adc_convert[I_CHASSIS], g_power_ctrl.power_pid.Output);

    /**
     * @brief 每个 CTRL 周期独立更新电压资格，不受充放电请求或环间胜负影响。
     * @note 严格高于 25.65 V 进入，严格低于 24.65 V 退出；区间及等于边界时保持。
     *       该历史仅由 ADC 更新，不表示本周期采用电压环。
     */
    if(g_power_ctrl.voltage_compete_enabled != 0U)
    {
        if(g_bsp_adc_feedback.adc_convert[V_CAP] < POWER_CTRL_VOLTAGE_COMPETE_V)
            g_power_ctrl.voltage_compete_enabled = 0U;
    }
    else if(g_bsp_adc_feedback.adc_convert[V_CAP] > POWER_CTRL_VOLTAGE_COMPETE_ENTER_V)
    {
        g_power_ctrl.voltage_compete_enabled = 1U;
    }

    /**
     * @brief 正目标电流、资格允许且电压候选更小时采用电压环，相等时采用电流环。
     * @note 只在控制权变化时用原始前馈加已算出的 ITerm 重置接管环的累计基值。
     *       竞争只判定一次，预置后的实际输出可能不同于比较候选，不保证切换无扰。
     */
    if(g_power_ctrl.power_pid.Output > 0.0f
       && g_power_ctrl.voltage_pid.Output < g_power_ctrl.current_pid.Output
       && g_power_ctrl.voltage_compete_enabled != 0U)
    {
        if(g_power_ctrl.voltage_loop_active == 0U)
        {
            g_power_ctrl.voltage_pid.Output = g_power_ctrl.f_generalized_duty
                + g_power_ctrl.voltage_pid.ITerm;
        }
        g_power_ctrl.voltage_loop_active = 1U;
        g_power_ctrl.generalized_duty = g_power_ctrl.voltage_pid.Output;
    }
    else
    {
        if(g_power_ctrl.voltage_loop_active != 0U)
        {
            g_power_ctrl.current_pid.Output = g_power_ctrl.f_generalized_duty
                + g_power_ctrl.current_pid.ITerm;
        }
        g_power_ctrl.voltage_loop_active = 0U;
        g_power_ctrl.generalized_duty = g_power_ctrl.current_pid.Output;
    }

    OUTPUT_LIMIT(g_power_ctrl.generalized_duty,
                 POWER_CTRL_OUTPUT_DUTY_MAX, BSP_HRTIM_GENERALIZED_DUTY_MIN);
}
/**
 * @brief 1 kHz 发布普通错误，避免在 ADC 控制周期内重复检查。
 * @note 局部快照保存 ADC 可抢占覆盖的反馈，不复用仍在运行的控制记录。
 *       电容低于最大电压的 10% 时保护，须外部充电达到下界，再按既有连续正常延时恢复。
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
        !(cap_voltage_v >= POWER_CTRL_CAP_MIN_VOLTAGE_V
          && cap_voltage_v <= POWER_CTRL_OVERVOLTAGE_V);
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
 * @pre 仅 ADC 上下文串行调用；f_generalized_duty 为本周期原始 Vcap/Vchassis。
 * @note 直接更新既有上下限成员；目标限幅不等同于实际电流或峰值保护。
 */
static void PowerCtrl_CalculateCurrentLimits(void)
{
    /* 低压时限制从电容提取功率，仍允许较小充电目标；5.8～11.6 V 连续恢复双向能力。
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
    g_power_ctrl.current_upper_limit_a *= g_power_ctrl.f_generalized_duty;
    g_power_ctrl.current_lower_limit_a *= g_power_ctrl.f_generalized_duty;
}

static void PowerCtrl_ResetLoops(void)
{
    g_power_ctrl.energy_pid.PID_reset(&g_power_ctrl.energy_pid);
    g_power_ctrl.power_pid.PID_reset(&g_power_ctrl.power_pid);
    g_power_ctrl.voltage_pid.PID_reset(&g_power_ctrl.voltage_pid);
    g_power_ctrl.current_pid.PID_reset(&g_power_ctrl.current_pid);
    /* 恢复时丢弃故障前资格和控制权历史，按电流环预置后计算首个 CTRL 周期。 */
    g_power_ctrl.voltage_compete_enabled = 0U;
    g_power_ctrl.voltage_loop_active = 0U;
    g_power_ctrl.current_upper_limit_a = 0.0f;
    g_power_ctrl.current_lower_limit_a = 0.0f;
    /* 保留本周期原始前馈和功率记录，供预置及限流换算使用；外环仍保持注释。 */
}
