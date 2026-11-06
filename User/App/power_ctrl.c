/**
 * @file power_ctrl.c
 * @brief 能量环、A 端功率环、电压竞争、电流环及自动恢复。
 * @details 正电流充电、负电流放电；逻辑 A 端为 chassis，本板对应 Timer B。
 * @note 所有写入串行且唯一；不在此实现采样、通信、中断或硬件过流保护。
 */
#include "power_ctrl.h"

#include "main.h"
#include "power_ctrl_config.h"
#include "bsp_adc.h"
#include "prot_judge.h"
#include "prot_supercap.h"
#include "bsp_hrtim.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#if (defined(POWER_CTRL_CLOSED_LOOP) + defined(POWER_CTRL_OPEN_LOOP)) != 1
#error Select_exactly_one_power_control_mode_in_main_h
#endif

PowerCtrl_t g_power_ctrl;

#if defined(POWER_CTRL_CLOSED_LOOP)
/** @brief 已在全部运行条件有效且完成平衡预置后开始等待，不以零时间戳作哨兵。 */
static bool s_restart_waiting;
/** @brief 闭环首次有效预置后的无错误等待起点 ms；错误时取消等待。 */
static uint32_t s_restart_start_ms;
static uint32_t s_now_ms;
#endif

static void PowerCtrl_ModeSwitch(void);
static void PowerCtrl_DutyCalcu(void);
static void PowerCtrl_UpdateErrors(void);
#if defined(POWER_CTRL_CLOSED_LOOP)
static void PowerCtrl_CascadePidCalcu(void);
static void PowerCtrl_UpdateErrorTimers(bool sample_usable);
static void PowerCtrl_ResetLoops(void);
static void PowerCtrl_ResetPid(PID_TypeDef *pid);
static float PowerCtrl_Clamp(float value, float lower, float upper);
static float PowerCtrl_ConstrainPid(PID_TypeDef *pid, const PID_TypeDef *before,
                                    float measure, float target, float lower, float upper);
#endif

/* ==================== 公共入口 ==================== */

void PowerCtrl_Init(void)
{
    BspHrtim_Init();
    memset(&g_power_ctrl, 0, sizeof(g_power_ctrl));
    g_power_ctrl.state = PROTECT_MODE;

#if defined(POWER_CTRL_CLOSED_LOOP)
    s_restart_waiting = false;
    s_restart_start_ms = 0U;

    /**
     * @brief 每次调用的离散 PID 系数，0 为待调试模板，不沿用旧板系数。
     * @details Kp 单位依次为 W/J、A/W、A/V、1/A；Ki 和 Kd 按原库逐次
     *          积分/差分，不自动乘除 dt。所有环每个控制周期计算一次。
     */
    /* 参照 Taurus Arm_PID_Init 的逐环调用格式；先清零再执行原库初始化。 */
    PID_Init(&g_power_ctrl.energy_pid, -POWER_CTRL_POWER_MIN_W, -POWER_CTRL_POWER_MIN_W, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, Integral_Limit);
    PID_Init(&g_power_ctrl.power_pid, POWER_CTRL_CAP_CURRENT_MAX_A,
             POWER_CTRL_CAP_CURRENT_MAX_A, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, Integral_Limit);
    PID_Init(&g_power_ctrl.voltage_pid, POWER_CTRL_CAP_CURRENT_MAX_A,
             POWER_CTRL_CAP_CURRENT_MAX_A, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, Integral_Limit);
    PID_Init(&g_power_ctrl.current_pid, BSP_HRTIM_GENERALIZED_DUTY_MAX,
             BSP_HRTIM_GENERALIZED_DUTY_MAX, 0.0f,
             0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, Integral_Limit);
#endif
}

void PowerCtrl_Task(void)
{
#if defined(POWER_CTRL_CLOSED_LOOP)
    s_now_ms = HAL_GetTick();
#endif
    PowerCtrl_UpdateErrors();
    PowerCtrl_DutyCalcu();
}

/* ==================== 状态切换 ==================== */

/**
 * @brief 有错误保持关闭；闭环等待恢复，开环在首个采样回调及远程解除后直接开启。
 * @pre 本周期错误已检查；闭环前馈已检查且等待起点由有效平衡预置记录。
 * @note 开环沿用 CubeMX 固定 50% 比较值，首次 ADC1 TC 前已运行多个 PWM 周期。
 */
static void PowerCtrl_ModeSwitch(void)
{
    if(g_power_ctrl.errors.all != 0U)
    {
        g_power_ctrl.state = PROTECT_MODE;
#if defined(POWER_CTRL_CLOSED_LOOP)
        s_restart_waiting = false;
#endif
    }
    else if(g_power_ctrl.state == PROTECT_MODE
#if defined(POWER_CTRL_CLOSED_LOOP)
            && s_restart_waiting
            && (uint32_t)(s_now_ms - s_restart_start_ms) >= POWER_CTRL_RESTART_DELAY_MS
#endif
    )
    {
#if defined(POWER_CTRL_CLOSED_LOOP)
        /* 保护期间已持续装载平衡点；复位保留本周期前馈，随后立即执行控制。 */
        PowerCtrl_ResetLoops();
        s_restart_waiting = false;
#endif
        g_power_ctrl.state = CTRL_MODE;
        BspHrtim_EnableOutput();
    }
    else if(g_power_ctrl.state != PROTECT_MODE && g_power_ctrl.state != CTRL_MODE)
    {
        g_power_ctrl.state = PROTECT_MODE;
#if defined(POWER_CTRL_CLOSED_LOOP)
        s_restart_waiting = false;
#endif
    }
}

/* ==================== 占空比计算与输出 ==================== */

/**
 * @brief 闭环持续预置前馈并执行控制；开环只切换输出，不改写固定比较值。
 * @pre 已完成错误检查；仅由固定周期 Task 串行调用，计数器及预装载持续运行。
 */
static void PowerCtrl_DutyCalcu(void)
{
#if defined(POWER_CTRL_CLOSED_LOOP)
    if(!g_power_ctrl.errors.bits.sample
       && g_bsp_adc_feedback.adc_convert[V_CAP] > 0.0f
       && g_bsp_adc_feedback.adc_convert[V_CHASSIS] > 0.0f)
    {
        g_power_ctrl.generalized_duty = g_bsp_adc_feedback.adc_convert[V_CAP]
            / g_bsp_adc_feedback.adc_convert[V_CHASSIS];
        if(!(fabsf(g_power_ctrl.generalized_duty) <= FLT_MAX))
            g_power_ctrl.errors.bits.numeric = 1U;
    }
    /* 先检查本周期前馈，防止比值溢出时仍按旧等待结果开启输出。 */
#endif
    PowerCtrl_ModeSwitch();

    // clang-format off
    switch(g_power_ctrl.state)
    {
        case PROTECT_MODE:
        {
            BspHrtim_DisableOutput();
#if defined(POWER_CTRL_CLOSED_LOOP)
            if(!g_power_ctrl.errors.bits.sample && !g_power_ctrl.errors.bits.numeric)
            {
                BspHrtim_SetGeneralizedDuty(g_power_ctrl.generalized_duty);
                if(g_power_ctrl.errors.all == 0U && !s_restart_waiting)
                {
                    /* 首次启动和故障恢复共用此处；初始化耗时不计入等待。 */
                    s_restart_start_ms = HAL_GetTick();
                    s_restart_waiting = true;
                }
            }
#endif
        }    break;

        case CTRL_MODE:
        {
#if defined(POWER_CTRL_CLOSED_LOOP)
            PowerCtrl_CascadePidCalcu();
            if(fabsf(g_power_ctrl.generalized_duty) <= FLT_MAX)
            {
                BspHrtim_SetGeneralizedDuty(g_power_ctrl.generalized_duty);
            }
            else
            {
                g_power_ctrl.errors.bits.numeric = 1U;
                BspHrtim_DisableOutput();
                g_power_ctrl.state = PROTECT_MODE;
                s_restart_waiting = false;
            }
#endif
        }    break;

        default:
        {
            g_power_ctrl.errors.bits.config = 1U;
            BspHrtim_DisableOutput();
            g_power_ctrl.state = PROTECT_MODE;
#if defined(POWER_CTRL_CLOSED_LOOP)
            s_restart_waiting = false;
#endif
        }    break;
    }
    // clang-format on
}

#if defined(POWER_CTRL_CLOSED_LOOP)
/**
 * @brief 正常控制的能量、功率、电压竞争和电流串级计算，不操作硬件输出。
 * @details 各级直接使用 PID 的 Output；保留抗饱和重算需要的计算前快照。
 *          电流目标暂存至外环抗饱和判断完成，竞争结果只记录本次重算前的比较。
 *          电压环 Output 表示允许的正向电流 A，未竞争时取当前电流上限。
 *          电流环 Output 为广义占空比修正，叠加 Vcap/Vchassis 前馈后限幅。
 * @pre 反馈已通过本周期有效性及数值检查，generalized_duty 保存本周期电压比前馈。
 */
static void PowerCtrl_CascadePidCalcu(void)
{
    g_power_ctrl.current_limit_a = POWER_CTRL_CAP_CURRENT_MAX_A * g_power_ctrl.generalized_duty;
    g_power_ctrl.power_feedback_w = g_bsp_adc_feedback.adc_convert[V_CHASSIS]
        * g_bsp_adc_feedback.adc_convert[I_CHASSIS];

    PID_TypeDef energy_before = g_power_ctrl.energy_pid;
    /* 重算可能改变功率环输出；外环仍需知道重算前是否受电压环限制。 */
    bool voltage_competing = false;
    /* 能量修正必须能超过裁判功率上限，才能请求负功率放电。 */
    g_power_ctrl.energy_pid.MaxOut = robot_status.chassis_power_limit - POWER_CTRL_POWER_MIN_W;
    g_power_ctrl.energy_pid.IntegralLimit = g_power_ctrl.energy_pid.MaxOut;
    PID_Calculate(&g_power_ctrl.energy_pid, power_heat_data.buffer_energy,
                  POWER_CTRL_ENERGY_TARGET_J);
    g_power_ctrl.energy_pid.Output = PowerCtrl_ConstrainPid(
        &g_power_ctrl.energy_pid, &energy_before,
        power_heat_data.buffer_energy, POWER_CTRL_ENERGY_TARGET_J,
        robot_status.chassis_power_limit - fminf(robot_status.chassis_power_limit,
            POWER_CTRL_CAP_CURRENT_MAX_A * g_bsp_adc_feedback.adc_convert[V_CAP]),
        robot_status.chassis_power_limit - fmaxf(POWER_CTRL_POWER_MIN_W,
            -POWER_CTRL_CAP_CURRENT_MAX_A * g_bsp_adc_feedback.adc_convert[V_CAP]));
    g_power_ctrl.power_target_w = PowerCtrl_Clamp(
        robot_status.chassis_power_limit - g_power_ctrl.energy_pid.Output,
        fmaxf(POWER_CTRL_POWER_MIN_W,
              -POWER_CTRL_CAP_CURRENT_MAX_A * g_bsp_adc_feedback.adc_convert[V_CAP]),
        fminf(robot_status.chassis_power_limit,
              POWER_CTRL_CAP_CURRENT_MAX_A * g_bsp_adc_feedback.adc_convert[V_CAP]));

    PID_TypeDef power_before = g_power_ctrl.power_pid;
    g_power_ctrl.power_pid.MaxOut = g_power_ctrl.current_limit_a;
    g_power_ctrl.power_pid.IntegralLimit = g_power_ctrl.current_limit_a;
    PID_Calculate(&g_power_ctrl.power_pid, g_power_ctrl.power_feedback_w,
                  g_power_ctrl.power_target_w);
    if(g_bsp_adc_feedback.adc_convert[V_CAP] > POWER_CTRL_VOLTAGE_COMPETE_V)
    {
        PID_TypeDef voltage_before = g_power_ctrl.voltage_pid;
        g_power_ctrl.voltage_pid.MaxOut = g_power_ctrl.current_limit_a;
        g_power_ctrl.voltage_pid.IntegralLimit = g_power_ctrl.current_limit_a;
        PID_Calculate(&g_power_ctrl.voltage_pid,
            g_bsp_adc_feedback.adc_convert[V_CAP], POWER_CTRL_CAP_TARGET_V);
        g_power_ctrl.voltage_pid.Output = PowerCtrl_ConstrainPid(
            &g_power_ctrl.voltage_pid, &voltage_before,
            g_bsp_adc_feedback.adc_convert[V_CAP], POWER_CTRL_CAP_TARGET_V,
            0.0f, g_power_ctrl.current_limit_a);
        if(g_bsp_adc_feedback.adc_convert[V_CAP] >= POWER_CTRL_CAP_TARGET_V
           && fabsf(g_power_ctrl.voltage_pid.Output) <= FLT_MAX)
            g_power_ctrl.voltage_pid.Output = 0.0f;
        voltage_competing = g_power_ctrl.power_pid.Output > g_power_ctrl.voltage_pid.Output;
    }
    else
    {
        PowerCtrl_ResetPid(&g_power_ctrl.voltage_pid);
        g_power_ctrl.voltage_pid.Output = g_power_ctrl.current_limit_a;
    }
    /* 暂存限幅结果，避免覆盖外环抗饱和判断仍需读取的功率环原始输出。 */
    float current_target_a = PowerCtrl_ConstrainPid(
        &g_power_ctrl.power_pid, &power_before,
        g_power_ctrl.power_feedback_w, g_power_ctrl.power_target_w,
        -g_power_ctrl.current_limit_a, g_power_ctrl.voltage_pid.Output);

    /* 在保存限幅结果前使用功率环原始输出，保留外环抗积分饱和判断。 */
    if(((g_power_ctrl.power_pid.Output > g_power_ctrl.voltage_pid.Output
         || voltage_competing)
        && g_power_ctrl.energy_pid.Iout < energy_before.Iout)
       || (g_power_ctrl.power_pid.Output < -g_power_ctrl.current_limit_a
           && g_power_ctrl.energy_pid.Iout > energy_before.Iout))
        g_power_ctrl.energy_pid.Iout = energy_before.Iout;
    g_power_ctrl.power_pid.Output = current_target_a;

    PID_TypeDef current_before = g_power_ctrl.current_pid;
    PID_Calculate(&g_power_ctrl.current_pid,
        g_bsp_adc_feedback.adc_convert[I_CHASSIS], g_power_ctrl.power_pid.Output);
    g_power_ctrl.current_pid.Output = PowerCtrl_ConstrainPid(
        &g_power_ctrl.current_pid, &current_before,
        g_bsp_adc_feedback.adc_convert[I_CHASSIS], g_power_ctrl.power_pid.Output,
        BSP_HRTIM_GENERALIZED_DUTY_MIN - g_power_ctrl.generalized_duty,
        BSP_HRTIM_GENERALIZED_DUTY_MAX - g_power_ctrl.generalized_duty);

    /* 内环已到占空比边界时，外环停止积累同向请求。 */
    if((g_power_ctrl.current_pid.Output >= BSP_HRTIM_GENERALIZED_DUTY_MAX
            - g_power_ctrl.generalized_duty
        && g_power_ctrl.power_pid.Iout > power_before.Iout)
       || (g_power_ctrl.current_pid.Output <= BSP_HRTIM_GENERALIZED_DUTY_MIN
            - g_power_ctrl.generalized_duty
           && g_power_ctrl.power_pid.Iout < power_before.Iout))
        g_power_ctrl.power_pid.Iout = power_before.Iout;
    if((g_power_ctrl.current_pid.Output >= BSP_HRTIM_GENERALIZED_DUTY_MAX
            - g_power_ctrl.generalized_duty
        && g_power_ctrl.energy_pid.Iout < energy_before.Iout)
       || (g_power_ctrl.current_pid.Output <= BSP_HRTIM_GENERALIZED_DUTY_MIN
            - g_power_ctrl.generalized_duty
           && g_power_ctrl.energy_pid.Iout > energy_before.Iout))
        g_power_ctrl.energy_pid.Iout = energy_before.Iout;
    g_power_ctrl.generalized_duty = PowerCtrl_Clamp(
        g_power_ctrl.generalized_duty + g_power_ctrl.current_pid.Output,
        BSP_HRTIM_GENERALIZED_DUTY_MIN, BSP_HRTIM_GENERALIZED_DUTY_MAX);
}
#endif

/* ==================== 私有辅助 ==================== */

static void PowerCtrl_UpdateErrors(void)
{
    /* 保留位从零开始；开环只检查远程请求、PWM 就绪及状态合法性。 */
    PowerCtrlError_t errors = {0};

    if(g_supercap_can.chassis_mode == SUPERCAP_CAN_REMOTE_PROTECT)
        errors.bits.remote = 1U;
    if(POWER_CTRL_PWM_READY != 1U
       || (g_power_ctrl.state != PROTECT_MODE && g_power_ctrl.state != CTRL_MODE))
        errors.bits.config = 1U;
#if defined(POWER_CTRL_CLOSED_LOOP)
    errors.bits.short_circuit = g_power_ctrl.errors.bits.short_circuit;
    if(POWER_CTRL_PARAMETERS_READY != 1U)
        errors.bits.config = 1U;
    if(g_adc_calibration_state.board_errors.no_record
       || g_adc_calibration_state.board_errors.duplicate_uid
       || g_adc_calibration_state.board_errors.invalid_id
       || g_adc_calibration_state.valid_channel_mask != ADC_CAL_ALL_CHANNELS_MASK)
        errors.bits.calibration = 1U;

    for(uint32_t channel = 0U; channel < ADC_CHANNEL_COUNT; channel++)
        if(!(fabsf(g_bsp_adc_feedback.adc_convert[channel]) <= FLT_MAX))
            errors.bits.sample = 1U;
    if(errors.bits.sample || g_bsp_adc_feedback.adc_convert[V_CAP] <= 0.0f
       || g_bsp_adc_feedback.adc_convert[V_CHASSIS] <= 0.0f
       || g_bsp_adc_feedback.adc_convert[V_BAT] <= 0.0f)
        errors.bits.numeric = 1U;

    /* 无校准或非有限反馈不得新锁存短路；既有短路位不在运行期清除。 */
    if(!errors.bits.calibration && !errors.bits.sample
       && g_bsp_adc_feedback.adc_convert[V_CHASSIS] < POWER_CTRL_SHORT_CIRCUIT_VOLTAGE_V
       && fabsf(g_bsp_adc_feedback.adc_convert[I_CHASSIS]) > POWER_CTRL_SHORT_CIRCUIT_CURRENT_A)
        errors.bits.short_circuit = 1U;

    PowerCtrl_UpdateErrorTimers(!errors.bits.sample);
    if(!errors.bits.sample)
    {
        if(g_power_ctrl.cap_overvoltage_cnt >= POWER_CTRL_CAP_OVERVOLTAGE_CYCLES)
            errors.bits.cap_overvoltage = 1U;
        if(g_power_ctrl.bat_undervoltage_cnt >= POWER_CTRL_BAT_UNDERVOLTAGE_CYCLES)
            errors.bits.bat_undervoltage = 1U;
        if(g_power_ctrl.chassis_overvoltage_cnt >= POWER_CTRL_CHASSIS_OVERVOLTAGE_CYCLES
           || (g_power_ctrl.errors.bits.chassis_overvoltage
               && g_bsp_adc_feedback.adc_convert[V_CHASSIS] >= POWER_CTRL_CHASSIS_RECOVER_V))
            errors.bits.chassis_overvoltage = 1U;
    }
    else
    {
        /* 无效数据不能解除已有物理故障，尤其是底盘过压迟滞。 */
        errors.bits.cap_overvoltage = g_power_ctrl.errors.bits.cap_overvoltage;
        errors.bits.chassis_overvoltage = g_power_ctrl.errors.bits.chassis_overvoltage;
        errors.bits.bat_undervoltage = g_power_ctrl.errors.bits.bat_undervoltage;
    }
    /* 两类报文必须分别收到且保持新鲜，不能由另一类报文刷新时间。
     * 功率 W 与缓冲能量 J 均来自 uint16_t 字段，天然有限且非负。 */
    if(!g_judge_robot_status_rx.valid || !g_judge_power_heat_rx.valid
       || (uint32_t)(s_now_ms - g_judge_robot_status_rx.update_ms) > POWER_CTRL_REFEREE_TIMEOUT_MS
       || (uint32_t)(s_now_ms - g_judge_power_heat_rx.update_ms) > POWER_CTRL_REFEREE_TIMEOUT_MS)
        errors.bits.referee = 1U;
#endif
    g_power_ctrl.errors = errors;
}

#if defined(POWER_CTRL_CLOSED_LOOP)
static void PowerCtrl_ResetLoops(void)
{
    PowerCtrl_ResetPid(&g_power_ctrl.energy_pid);
    PowerCtrl_ResetPid(&g_power_ctrl.power_pid);
    PowerCtrl_ResetPid(&g_power_ctrl.voltage_pid);
    PowerCtrl_ResetPid(&g_power_ctrl.current_pid);
    g_power_ctrl.power_target_w = 0.0f;
    g_power_ctrl.current_limit_a = 0.0f;
    /* generalized_duty 是本周期已检查的前馈，进入 CTRL 时仍需叠加 PID 修正。 */
}

static void PowerCtrl_ResetPid(PID_TypeDef *pid)
{
    PID_TypeDef config = *pid;
    memset(pid, 0, sizeof(*pid));
    PID_Init(pid, config.MaxOut, config.IntegralLimit, config.DeadBand,
        config.Kp, config.Ki, config.Kd, config.ScalarA, config.ScalarB,
        config.Output_Filtering_Coefficient, config.Derivative_Filtering_Coefficient, config.Improve);
}

/**
 * @brief 每个 25 kHz 控制周期累计连续异常次数，条件解除或采样无效时清零。
 * @param[in] sample_usable 本周期五路采样是否均为有限数值。
 * @note 达到各自阈值后保持计数，避免长时间故障使计数回绕；计数单位不是毫秒。
 *       无效采样仅清除连续计数，已有物理故障由错误判断保持。
 */
static void PowerCtrl_UpdateErrorTimers(bool sample_usable)
{
    if(sample_usable
       && g_power_ctrl.errors.bits.cap_overvoltage
       && g_bsp_adc_feedback.adc_convert[V_CAP] > POWER_CTRL_CAP_OVERVOLTAGE_V)
        g_power_ctrl.cap_overvoltage_cnt = POWER_CTRL_CAP_OVERVOLTAGE_CYCLES;
    if(sample_usable
       && g_power_ctrl.errors.bits.bat_undervoltage
       && g_bsp_adc_feedback.adc_convert[V_BAT] < POWER_CTRL_BAT_UNDERVOLTAGE_V)
        g_power_ctrl.bat_undervoltage_cnt = POWER_CTRL_BAT_UNDERVOLTAGE_CYCLES;

    g_power_ctrl.cap_overvoltage_cnt =
        (sample_usable && g_bsp_adc_feedback.adc_convert[V_CAP] > POWER_CTRL_CAP_OVERVOLTAGE_V)
        ? g_power_ctrl.cap_overvoltage_cnt
            + (g_power_ctrl.cap_overvoltage_cnt < POWER_CTRL_CAP_OVERVOLTAGE_CYCLES)
        : 0U;
    g_power_ctrl.chassis_overvoltage_cnt =
        (sample_usable
         && g_bsp_adc_feedback.adc_convert[V_CHASSIS] > POWER_CTRL_CHASSIS_OVERVOLTAGE_V)
        ? g_power_ctrl.chassis_overvoltage_cnt
            + (g_power_ctrl.chassis_overvoltage_cnt < POWER_CTRL_CHASSIS_OVERVOLTAGE_CYCLES)
        : 0U;
    g_power_ctrl.bat_undervoltage_cnt =
        (sample_usable && g_bsp_adc_feedback.adc_convert[V_BAT] < POWER_CTRL_BAT_UNDERVOLTAGE_V)
        ? g_power_ctrl.bat_undervoltage_cnt
            + (g_power_ctrl.bat_undervoltage_cnt < POWER_CTRL_BAT_UNDERVOLTAGE_CYCLES)
        : 0U;
}

static float PowerCtrl_Clamp(float value, float lower, float upper)
{
    if(value < lower)
        value = lower;
    else if(value > upper)
        value = upper;
    return value;
}

static float PowerCtrl_ConstrainPid(PID_TypeDef *pid, const PID_TypeDef *before,
                                    float measure, float target, float lower, float upper)
{
    /* 原库计算一次后，若本次积分继续推动外部饱和，则回到计算前状态，
     * 暂时以 Ki=0 重算同一次采样，保留以前的积分补偿，不修改库算法。 */
    float integral_delta = pid->Iout - before->Iout;
    if((pid->Output >= upper && integral_delta > 0.0f)
       || (pid->Output <= lower && integral_delta < 0.0f))
    {
        float ki = pid->Ki;
        float max_out = pid->MaxOut;
        float integral_limit = pid->IntegralLimit;
        *pid = *before;
        pid->MaxOut = max_out;
        pid->IntegralLimit = integral_limit;
        pid->Ki = 0.0f;
        PID_Calculate(pid, measure, target);
        pid->Ki = ki;
    }
    return PowerCtrl_Clamp(pid->Output, lower, upper);
}

#endif
