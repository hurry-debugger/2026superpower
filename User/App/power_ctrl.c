/**
 * @file power_ctrl.c
 * @brief 能量环、A 端功率环、电压竞争、电流环及自动恢复。
 * @details 正电流充电、负电流放电；逻辑 A 端为 chassis，本板对应 Timer B。
 * @note 所有写入串行且唯一；不在此实现采样、通信、中断或硬件过流保护。
 */
#include "power_ctrl.h"

#include "main.h"
#include "power_ctrl_config.h"
#include "bsp_adc_feedback.h"
#include "prot_judge.h"
#include "prot_supercap.h"
#include "bsp_hrtim.h"

#include <float.h>
#include <math.h>
#include <string.h>

#if (defined(POWER_CTRL_MATCH) + defined(POWER_CTRL_NO_REFEREE) + defined(POWER_CTRL_OPEN_LOOP)) != 1
#error Select_exactly_one_power_control_mode_in_main_h
#endif

PowerCtrl_t g_power_ctrl;

static bool s_preload_pending;
static uint32_t s_restart_start_ms;
static uint32_t s_preload_start_ms;
static uint32_t s_now_ms;
static uint32_t s_last_sample_sequence;
static uint32_t s_sample_age_us;
static bool s_sample_seen;
static bool s_timing_fault;

static void PowerCtrl_ModeSwitch(void);
static void PowerCtrl_DutyCalcu(PowerCtrlState_e previous_state);
#if !defined(POWER_CTRL_OPEN_LOOP)
static void PowerCtrl_CascadePidCalcu(void);
#endif
static void PowerCtrl_UpdateErrorTimers(bool sample_usable);
static void PowerCtrl_UpdateErrors(void);
static void PowerCtrl_EnterProtect(PowerCtrlState_e previous_state);
static void PowerCtrl_ResetLoops(void);
static void PowerCtrl_ResetPid(PID_TypeDef *pid);
static uint32_t PowerCtrl_UsToMs(uint32_t time_us);
static float PowerCtrl_Clamp(float value, float lower, float upper);
static float PowerCtrl_ConstrainPid(PID_TypeDef *pid, const PID_TypeDef *before,
                                    float measure, float target, float lower, float upper);
static bool PowerCtrl_ConfigValid(void);
static bool PowerCtrl_LoopNumbersValid(void);
static float PowerCtrl_CalculatePid(PID_TypeDef *pid, float measure, float target);

/* ==================== 公共入口 ==================== */

void PowerCtrl_Init(void)
{
    BspHrtim_Init();
    memset(&g_power_ctrl, 0, sizeof(g_power_ctrl));
    g_power_ctrl.state = PROTECT_MODE;
    g_power_ctrl.test_power_w = POWER_CTRL_TEST_POWER_W;
    s_preload_pending = false;
    s_restart_start_ms = 0U;
    s_preload_start_ms = 0U;
    s_sample_seen = false;
    s_last_sample_sequence = 0U;
    s_timing_fault = false;
    s_sample_age_us = 0U;

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
}

void PowerCtrl_Task(void)
{
    /* 状态快照仅用于处理进入保护、恢复状态时的一次性动作。 */
    PowerCtrlState_e previous_state = g_power_ctrl.state;
    s_now_ms = HAL_GetTick();
    PowerCtrl_UpdateErrors();
    PowerCtrl_ModeSwitch();
    PowerCtrl_DutyCalcu(previous_state);
}

void PowerCtrl_NotifyTimingFault(void)
{
    s_timing_fault = true;
    s_now_ms = HAL_GetTick();
    g_power_ctrl.errors |= POWER_CTRL_ERROR_TIMING;
    PowerCtrl_EnterProtect(g_power_ctrl.state);
}

/* ==================== 状态切换 ==================== */

/**
 * @brief 依据错误及恢复进度切换状态；只修改运行状态，不执行输出动作。
 */
static void PowerCtrl_ModeSwitch(void)
{
#if defined(POWER_CTRL_OPEN_LOOP)
    if(g_power_ctrl.errors != POWER_CTRL_ERROR_NONE || !g_power_ctrl.open_loop_enable
       || !(g_bsp_adc_feedback.adc_convert[V_CAP] < POWER_CTRL_CAP_TARGET_V))
#else
    if(g_power_ctrl.errors != POWER_CTRL_ERROR_NONE)
#endif
        g_power_ctrl.state = PROTECT_MODE;
    else if(g_power_ctrl.state == PROTECT_MODE)
        g_power_ctrl.state = RESTART_MODE;
    else if(g_power_ctrl.state == RESTART_MODE)
    {
        /* 多等一个 tick，覆盖写寄存器与毫秒边界的相位，不能只向上取整。 */
        if(g_power_ctrl.restart_elapsed_ms >= POWER_CTRL_RESTART_DELAY_MS
           && s_preload_pending
           && (uint32_t)(s_now_ms - s_preload_start_ms)
               >= PowerCtrl_UsToMs(POWER_CTRL_PWM_PERIOD_US) + 1U)
            g_power_ctrl.state = CTRL_MODE;
    }
    else if(g_power_ctrl.state != CTRL_MODE)
        g_power_ctrl.state = PROTECT_MODE;
}

/* ==================== 占空比计算与输出 ==================== */

/**
 * @brief 按状态执行关断、恢复预装载或正常控制，统一管理周期内的功率输出。
 * @param[in] previous_state 本周期切换前的状态，用于进入状态时复位环路或启动计时。
 * @pre 已完成错误检查及状态切换；仅由固定周期 Task 串行调用。
 */
static void PowerCtrl_DutyCalcu(PowerCtrlState_e previous_state)
{
    // clang-format off
    switch(g_power_ctrl.state)
    {
        case PROTECT_MODE:
        {
            PowerCtrl_EnterProtect(previous_state);
        }    break;

        case RESTART_MODE:
        {
            if(previous_state == PROTECT_MODE)
                s_restart_start_ms = s_now_ms;
            g_power_ctrl.restart_elapsed_ms = s_now_ms - s_restart_start_ms;
            if(g_power_ctrl.restart_elapsed_ms >= POWER_CTRL_RESTART_DELAY_MS
               && !s_preload_pending)
            {
                g_power_ctrl.generalized_duty = g_bsp_adc_feedback.adc_convert[V_CAP]
                    / g_bsp_adc_feedback.adc_convert[V_CHASSIS];
                BspHrtim_SetGeneralizedDuty(g_power_ctrl.generalized_duty);
                s_preload_start_ms = HAL_GetTick();
                s_preload_pending = true;
            }
        }    break;

        case CTRL_MODE:
        {
#if defined(POWER_CTRL_OPEN_LOOP)
            g_power_ctrl.generalized_duty = g_power_ctrl.open_loop_duty;
#else
            PowerCtrl_CascadePidCalcu();
#endif
            if(PowerCtrl_LoopNumbersValid())
            {
                BspHrtim_SetGeneralizedDuty(g_power_ctrl.generalized_duty);
                if(!g_power_ctrl.output_enabled)
                {
                    BspHrtim_EnableOutput();
                    g_power_ctrl.output_enabled = true;
                }
            }
            else
            {
                g_power_ctrl.errors |= POWER_CTRL_ERROR_NUMERIC;
                PowerCtrl_EnterProtect(g_power_ctrl.state);
            }
        }    break;

        default:
        {
            g_power_ctrl.errors |= POWER_CTRL_ERROR_CONFIG;
            PowerCtrl_EnterProtect(g_power_ctrl.state);
        }    break;
    }
    // clang-format on
}

#if !defined(POWER_CTRL_OPEN_LOOP)
/**
 * @brief 正常控制的能量、功率、电压竞争和电流串级计算，不操作硬件输出。
 * @details 各级直接使用 PID 的 Output；仅保留抗饱和重算需要的计算前快照。
 *          电压环 Output 表示允许的正向电流 A，未竞争时取当前电流上限。
 *          电流环 Output 为广义占空比修正，叠加 Vcap/Vchassis 前馈后限幅。
 * @pre 反馈已通过本周期有效性及数值检查，两个端口电压均为正。
 */
static void PowerCtrl_CascadePidCalcu(void)
{
    g_power_ctrl.current_limit_a = POWER_CTRL_CAP_CURRENT_MAX_A
        * g_bsp_adc_feedback.adc_convert[V_CAP] / g_bsp_adc_feedback.adc_convert[V_CHASSIS];
    g_power_ctrl.power_feedback_w = g_bsp_adc_feedback.adc_convert[V_CHASSIS]
        * g_bsp_adc_feedback.adc_convert[I_CHASSIS];

#if defined(POWER_CTRL_MATCH)
    PID_TypeDef energy_before = g_power_ctrl.energy_pid;
    /* 能量修正必须能超过裁判功率上限，才能请求负功率放电。 */
    g_power_ctrl.energy_pid.MaxOut = robot_status.chassis_power_limit - POWER_CTRL_POWER_MIN_W;
    g_power_ctrl.energy_pid.IntegralLimit = g_power_ctrl.energy_pid.MaxOut;
    PowerCtrl_CalculatePid(&g_power_ctrl.energy_pid, power_heat_data.buffer_energy,
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
#else
    g_power_ctrl.power_target_w = PowerCtrl_Clamp(g_power_ctrl.test_power_w,
        fmaxf(POWER_CTRL_POWER_MIN_W,
              -POWER_CTRL_CAP_CURRENT_MAX_A * g_bsp_adc_feedback.adc_convert[V_CAP]),
        POWER_CTRL_CAP_CURRENT_MAX_A * g_bsp_adc_feedback.adc_convert[V_CAP]);
#endif

    PID_TypeDef power_before = g_power_ctrl.power_pid;
    g_power_ctrl.power_pid.MaxOut = g_power_ctrl.current_limit_a;
    g_power_ctrl.power_pid.IntegralLimit = g_power_ctrl.current_limit_a;
    PowerCtrl_CalculatePid(&g_power_ctrl.power_pid, g_power_ctrl.power_feedback_w,
                          g_power_ctrl.power_target_w);
    g_power_ctrl.voltage_competing = false;
    if(g_bsp_adc_feedback.adc_convert[V_CAP] > POWER_CTRL_VOLTAGE_COMPETE_V)
    {
        PID_TypeDef voltage_before = g_power_ctrl.voltage_pid;
        g_power_ctrl.voltage_pid.MaxOut = g_power_ctrl.current_limit_a;
        g_power_ctrl.voltage_pid.IntegralLimit = g_power_ctrl.current_limit_a;
        PowerCtrl_CalculatePid(&g_power_ctrl.voltage_pid,
            g_bsp_adc_feedback.adc_convert[V_CAP], POWER_CTRL_CAP_TARGET_V);
        g_power_ctrl.voltage_pid.Output = PowerCtrl_ConstrainPid(
            &g_power_ctrl.voltage_pid, &voltage_before,
            g_bsp_adc_feedback.adc_convert[V_CAP], POWER_CTRL_CAP_TARGET_V,
            0.0f, g_power_ctrl.current_limit_a);
        if(g_bsp_adc_feedback.adc_convert[V_CAP] >= POWER_CTRL_CAP_TARGET_V
           && isfinite(g_power_ctrl.voltage_pid.Output))
            g_power_ctrl.voltage_pid.Output = 0.0f;
        g_power_ctrl.voltage_competing =
            g_power_ctrl.power_pid.Output > g_power_ctrl.voltage_pid.Output;
    }
    else
    {
        PowerCtrl_ResetPid(&g_power_ctrl.voltage_pid);
        g_power_ctrl.voltage_pid.Output = g_power_ctrl.current_limit_a;
    }
    g_power_ctrl.current_target_a = PowerCtrl_ConstrainPid(
        &g_power_ctrl.power_pid, &power_before,
        g_power_ctrl.power_feedback_w, g_power_ctrl.power_target_w,
        -g_power_ctrl.current_limit_a, g_power_ctrl.voltage_pid.Output);

#if defined(POWER_CTRL_MATCH)
    /* 在保存限幅结果前使用功率环原始输出，保留外环抗积分饱和判断。 */
    if(((g_power_ctrl.power_pid.Output > g_power_ctrl.voltage_pid.Output
         || g_power_ctrl.voltage_competing)
        && g_power_ctrl.energy_pid.Iout < energy_before.Iout)
       || (g_power_ctrl.power_pid.Output < -g_power_ctrl.current_limit_a
           && g_power_ctrl.energy_pid.Iout > energy_before.Iout))
        g_power_ctrl.energy_pid.Iout = energy_before.Iout;
#endif
    g_power_ctrl.power_pid.Output = g_power_ctrl.current_target_a;

    PID_TypeDef current_before = g_power_ctrl.current_pid;
    PowerCtrl_CalculatePid(&g_power_ctrl.current_pid,
        g_bsp_adc_feedback.adc_convert[I_CHASSIS], g_power_ctrl.power_pid.Output);
    g_power_ctrl.current_pid.Output = PowerCtrl_ConstrainPid(
        &g_power_ctrl.current_pid, &current_before,
        g_bsp_adc_feedback.adc_convert[I_CHASSIS], g_power_ctrl.power_pid.Output,
        BSP_HRTIM_GENERALIZED_DUTY_MIN - g_bsp_adc_feedback.adc_convert[V_CAP]
            / g_bsp_adc_feedback.adc_convert[V_CHASSIS],
        BSP_HRTIM_GENERALIZED_DUTY_MAX - g_bsp_adc_feedback.adc_convert[V_CAP]
            / g_bsp_adc_feedback.adc_convert[V_CHASSIS]);

    /* 内环已到占空比边界时，外环停止积累同向请求。 */
    if((g_power_ctrl.current_pid.Output >= BSP_HRTIM_GENERALIZED_DUTY_MAX
            - g_bsp_adc_feedback.adc_convert[V_CAP] / g_bsp_adc_feedback.adc_convert[V_CHASSIS]
        && g_power_ctrl.power_pid.Iout > power_before.Iout)
       || (g_power_ctrl.current_pid.Output <= BSP_HRTIM_GENERALIZED_DUTY_MIN
            - g_bsp_adc_feedback.adc_convert[V_CAP] / g_bsp_adc_feedback.adc_convert[V_CHASSIS]
           && g_power_ctrl.power_pid.Iout < power_before.Iout))
        g_power_ctrl.power_pid.Iout = power_before.Iout;
#if defined(POWER_CTRL_MATCH)
    if((g_power_ctrl.current_pid.Output >= BSP_HRTIM_GENERALIZED_DUTY_MAX
            - g_bsp_adc_feedback.adc_convert[V_CAP] / g_bsp_adc_feedback.adc_convert[V_CHASSIS]
        && g_power_ctrl.energy_pid.Iout < energy_before.Iout)
       || (g_power_ctrl.current_pid.Output <= BSP_HRTIM_GENERALIZED_DUTY_MIN
            - g_bsp_adc_feedback.adc_convert[V_CAP] / g_bsp_adc_feedback.adc_convert[V_CHASSIS]
           && g_power_ctrl.energy_pid.Iout > energy_before.Iout))
        g_power_ctrl.energy_pid.Iout = energy_before.Iout;
#endif
    g_power_ctrl.generalized_duty = PowerCtrl_Clamp(
        g_bsp_adc_feedback.adc_convert[V_CAP] / g_bsp_adc_feedback.adc_convert[V_CHASSIS]
            + g_power_ctrl.current_pid.Output,
        BSP_HRTIM_GENERALIZED_DUTY_MIN, BSP_HRTIM_GENERALIZED_DUTY_MAX);
}
#endif

/* ==================== 私有辅助 ==================== */

static void PowerCtrl_UpdateErrors(void)
{
    uint32_t errors = POWER_CTRL_ERROR_NONE;
    if(s_timing_fault)
        errors |= POWER_CTRL_ERROR_TIMING;
    if(g_supercap_can.chassis_mode == SUPERCAP_CAN_REMOTE_PROTECT)
        errors |= POWER_CTRL_ERROR_REMOTE;
    if(!PowerCtrl_ConfigValid()
       || (g_power_ctrl.state != PROTECT_MODE && g_power_ctrl.state != RESTART_MODE
           && g_power_ctrl.state != CTRL_MODE))
        errors |= POWER_CTRL_ERROR_CONFIG;
    if(g_adc_calibration_state.board_errors != 0U
       || g_adc_calibration_state.valid_channel_mask != ADC_CAL_ALL_CHANNELS_MASK)
        errors |= POWER_CTRL_ERROR_CALIBRATION;

    bool finite_sample = true;
    for(uint32_t channel = 0U; channel < ADC_CHANNEL_COUNT; channel++)
        if(!isfinite(g_bsp_adc_feedback.adc_convert[channel]))
            finite_sample = false;
    /* 在唯一固定周期上下文用发布序号检测微秒级停更，毫秒时间用于长时间陈旧检查。
     * 截止时间在第一个超过两倍发布周期的控制调用中检出，不依赖毫秒 tick 分辨率。 */
    if(!s_sample_seen || s_last_sample_sequence != g_bsp_adc_feedback.sequence)
    {
        s_last_sample_sequence = g_bsp_adc_feedback.sequence;
        s_sample_seen = true;
        s_sample_age_us = 0U;
    }
    else if(s_sample_age_us <= UINT32_MAX - POWER_CTRL_PERIOD_US)
        s_sample_age_us += POWER_CTRL_PERIOD_US;
    else
        s_sample_age_us = UINT32_MAX;
    bool sample_usable = finite_sample && g_bsp_adc_feedback.valid
        && s_sample_age_us <= 2U * POWER_CTRL_SAMPLE_PERIOD_US
        && (uint32_t)(s_now_ms - g_bsp_adc_feedback.update_ms)
            <= PowerCtrl_UsToMs(2U * POWER_CTRL_SAMPLE_PERIOD_US);
    if(!sample_usable)
        errors |= POWER_CTRL_ERROR_SAMPLE;
    if(!finite_sample || g_bsp_adc_feedback.adc_convert[V_CAP] <= 0.0f
       || g_bsp_adc_feedback.adc_convert[V_CHASSIS] <= 0.0f
       || g_bsp_adc_feedback.adc_convert[V_BAT] <= 0.0f)
        errors |= POWER_CTRL_ERROR_NUMERIC;

    PowerCtrl_UpdateErrorTimers(sample_usable);
    if(sample_usable)
    {
        if(g_power_ctrl.cap_overvoltage_cnt >= POWER_CTRL_CAP_OVERVOLTAGE_CYCLES)
            errors |= POWER_CTRL_ERROR_CAP_OVERVOLTAGE;
        if(g_power_ctrl.bat_undervoltage_cnt >= POWER_CTRL_BAT_UNDERVOLTAGE_CYCLES)
            errors |= POWER_CTRL_ERROR_BAT_UNDERVOLTAGE;
        if(g_power_ctrl.chassis_overvoltage_cnt >= POWER_CTRL_CHASSIS_OVERVOLTAGE_CYCLES
           || ((g_power_ctrl.errors & POWER_CTRL_ERROR_CHASSIS_OVERVOLTAGE) != 0U
               && g_bsp_adc_feedback.adc_convert[V_CHASSIS] >= POWER_CTRL_CHASSIS_RECOVER_V))
            errors |= POWER_CTRL_ERROR_CHASSIS_OVERVOLTAGE;
    }
    else
    {
        /* 无效数据不能解除已有物理故障，尤其是底盘过压迟滞。 */
        errors |= g_power_ctrl.errors & (POWER_CTRL_ERROR_CAP_OVERVOLTAGE
            | POWER_CTRL_ERROR_CHASSIS_OVERVOLTAGE | POWER_CTRL_ERROR_BAT_UNDERVOLTAGE);
    }
#if defined(POWER_CTRL_MATCH)
    /* 两类报文必须分别收到且保持新鲜，不能由另一类报文刷新时间。
     * 功率 W 与缓冲能量 J 均来自 uint16_t 字段，天然有限且非负。 */
    if(!g_judge_robot_status_rx.valid || !g_judge_power_heat_rx.valid
       || (uint32_t)(s_now_ms - g_judge_robot_status_rx.update_ms) > POWER_CTRL_REFEREE_TIMEOUT_MS
       || (uint32_t)(s_now_ms - g_judge_power_heat_rx.update_ms) > POWER_CTRL_REFEREE_TIMEOUT_MS)
        errors |= POWER_CTRL_ERROR_REFEREE;
#elif defined(POWER_CTRL_NO_REFEREE)
    if(!isfinite(g_power_ctrl.test_power_w))
        errors |= POWER_CTRL_ERROR_NUMERIC;
#else
    if(g_power_ctrl.open_loop_enable && (!isfinite(g_power_ctrl.open_loop_duty)
       || g_power_ctrl.open_loop_duty < BSP_HRTIM_GENERALIZED_DUTY_MIN
       || g_power_ctrl.open_loop_duty > BSP_HRTIM_GENERALIZED_DUTY_MAX))
        errors |= POWER_CTRL_ERROR_NUMERIC;
#endif
    if((errors & POWER_CTRL_ERROR_NUMERIC) == 0U)
    {
        float ratio = g_bsp_adc_feedback.adc_convert[V_CAP]
            / g_bsp_adc_feedback.adc_convert[V_CHASSIS];
        float cap_power = POWER_CTRL_CAP_CURRENT_MAX_A * g_bsp_adc_feedback.adc_convert[V_CAP];
        float current_limit = cap_power / g_bsp_adc_feedback.adc_convert[V_CHASSIS];
        if(!isfinite(ratio) || ratio < BSP_HRTIM_GENERALIZED_DUTY_MIN
           || ratio > BSP_HRTIM_GENERALIZED_DUTY_MAX || !isfinite(cap_power)
           || !isfinite(current_limit)
           || !isfinite(g_bsp_adc_feedback.adc_convert[V_CHASSIS]
                        * g_bsp_adc_feedback.adc_convert[I_CHASSIS]))
            errors |= POWER_CTRL_ERROR_NUMERIC;
    }
    g_power_ctrl.errors = errors;
}

/**
 * @brief 先关断输出，再按切换前的状态复位环路和恢复计时。
 * @param[in] previous_state 进入保护之前的状态，避免状态已切换后漏掉环路复位。
 * @note 时序超期和开环关闭入口直接调用，保持立即关断；周期入口经 DutyCalcu 调用。
 */
static void PowerCtrl_EnterProtect(PowerCtrlState_e previous_state)
{
    if(g_power_ctrl.output_enabled)
        BspHrtim_DisableOutput();
    g_power_ctrl.output_enabled = false;
    if(previous_state != PROTECT_MODE)
        PowerCtrl_ResetLoops();
    g_power_ctrl.state = PROTECT_MODE;
    g_power_ctrl.restart_elapsed_ms = 0U;
    s_restart_start_ms = s_now_ms;
    s_preload_pending = false;
}

static void PowerCtrl_ResetLoops(void)
{
    PowerCtrl_ResetPid(&g_power_ctrl.energy_pid);
    PowerCtrl_ResetPid(&g_power_ctrl.power_pid);
    PowerCtrl_ResetPid(&g_power_ctrl.voltage_pid);
    PowerCtrl_ResetPid(&g_power_ctrl.current_pid);
    g_power_ctrl.power_target_w = 0.0f;
    g_power_ctrl.current_target_a = 0.0f;
    g_power_ctrl.current_limit_a = 0.0f;
    g_power_ctrl.voltage_competing = false;
    g_power_ctrl.generalized_duty = 0.0f;
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
 * @param[in] sample_usable 本周期采样是否有限、有效且未超时。
 * @note 达到各自阈值后保持计数，避免长时间故障使计数回绕；计数单位不是毫秒。
 *       无效采样仅清除连续计数，已有物理故障由错误判断保持。
 */
static void PowerCtrl_UpdateErrorTimers(bool sample_usable)
{
    if(sample_usable
       && (g_power_ctrl.errors & POWER_CTRL_ERROR_CAP_OVERVOLTAGE) != 0U
       && g_bsp_adc_feedback.adc_convert[V_CAP] > POWER_CTRL_CAP_OVERVOLTAGE_V)
        g_power_ctrl.cap_overvoltage_cnt = POWER_CTRL_CAP_OVERVOLTAGE_CYCLES;
    if(sample_usable
       && (g_power_ctrl.errors & POWER_CTRL_ERROR_BAT_UNDERVOLTAGE) != 0U
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

static uint32_t PowerCtrl_UsToMs(uint32_t time_us)
{
    return time_us / 1000U + (time_us % 1000U != 0U ? 1U : 0U);
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
        PowerCtrl_CalculatePid(pid, measure, target);
        pid->Ki = ki;
    }
    return PowerCtrl_Clamp(pid->Output, lower, upper);
}

static bool PowerCtrl_ConfigValid(void)
{
    return POWER_CTRL_PARAMETERS_READY == 1U && POWER_CTRL_PWM_READY == 1U
        && isfinite(g_power_ctrl.energy_pid.Kp) && isfinite(g_power_ctrl.energy_pid.Ki)
        && isfinite(g_power_ctrl.energy_pid.Kd) && isfinite(g_power_ctrl.power_pid.Kp)
        && isfinite(g_power_ctrl.power_pid.Ki) && isfinite(g_power_ctrl.power_pid.Kd)
        && isfinite(g_power_ctrl.voltage_pid.Kp) && isfinite(g_power_ctrl.voltage_pid.Ki)
        && isfinite(g_power_ctrl.voltage_pid.Kd) && isfinite(g_power_ctrl.current_pid.Kp)
        && isfinite(g_power_ctrl.current_pid.Ki) && isfinite(g_power_ctrl.current_pid.Kd);
}

static bool PowerCtrl_LoopNumbersValid(void)
{
    return isfinite(g_power_ctrl.power_target_w) && isfinite(g_power_ctrl.power_feedback_w)
        && isfinite(g_power_ctrl.current_target_a) && isfinite(g_power_ctrl.current_limit_a)
        && isfinite(g_power_ctrl.generalized_duty)
        && isfinite(g_power_ctrl.energy_pid.Output) && isfinite(g_power_ctrl.energy_pid.Iout)
        && isfinite(g_power_ctrl.power_pid.Output) && isfinite(g_power_ctrl.power_pid.Iout)
        && isfinite(g_power_ctrl.voltage_pid.Output) && isfinite(g_power_ctrl.voltage_pid.Iout)
        && isfinite(g_power_ctrl.current_pid.Output) && isfinite(g_power_ctrl.current_pid.Iout);
}

static float PowerCtrl_CalculatePid(PID_TypeDef *pid, float measure, float target)
{
    /* 仅检查本应用启用的标准位置式与积分限幅路径，不改变原库计算。
     * 用 double 预查 float 乘法/差分/累加，防止原库削顶后掩盖 Inf。
     * 不声称双精度开销满足实时预算，目标执行时间仍需测量。 */
    double error = (double)target - (double)measure;
    double proportional = (double)pid->Kp * error;
    double integral = (double)pid->Ki * error;
    double difference = error - (double)pid->Last_Err;
    double derivative = (double)pid->Kd * difference;
    double next_integral = (double)pid->Iout + integral;
    double sum_pi = proportional + (double)pid->Iout;
    double sum_pid = sum_pi + derivative;
    double sum_next_pi = proportional + next_integral;
    double sum_next = sum_next_pi + derivative;
    if(!isfinite(error) || fabs(error) > FLT_MAX || !isfinite(proportional)
       || fabs(proportional) > FLT_MAX || !isfinite(integral) || fabs(integral) > FLT_MAX
       || !isfinite(difference) || fabs(difference) > FLT_MAX
       || !isfinite(derivative) || fabs(derivative) > FLT_MAX
       || !isfinite(next_integral) || fabs(next_integral) > FLT_MAX
       || !isfinite(sum_pi) || fabs(sum_pi) > FLT_MAX
       || !isfinite(sum_pid) || fabs(sum_pid) > FLT_MAX
       || !isfinite(sum_next_pi) || fabs(sum_next_pi) > FLT_MAX
       || !isfinite(sum_next) || fabs(sum_next) > FLT_MAX)
    {
        pid->Output = NAN;
    }
    else
        PID_Calculate(pid, measure, target);
    return pid->Output;
}

void PowerCtrl_SetTestPower(float power_w)
{
    g_power_ctrl.test_power_w = power_w;
}

void PowerCtrl_SetOpenLoopDuty(float generalized_duty)
{
    g_power_ctrl.open_loop_duty = generalized_duty;
}

void PowerCtrl_SetOpenLoopEnable(bool enable)
{
    g_power_ctrl.open_loop_enable = enable;
#if defined(POWER_CTRL_OPEN_LOOP)
    if(!enable)
        PowerCtrl_EnterProtect(g_power_ctrl.state);
#endif
}
