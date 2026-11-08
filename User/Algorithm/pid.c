/**
 * @file pid.c
 * @brief Taurus-arm PID 库的本工程副本，提供可选的外部限幅抗积分饱和。
 * @details 来源 Hongxi Wong V1.0.6；外部限幅抗饱和和位置式输出跟踪均由 Improve 显式启用。
 *          未启用跟踪的实例保留原算法；复位仅清运行状态并保留参数。
 * @note 系数按每次调用定义，不自动处理 dt。初始化前由调用者清零结构体；
 *       跟踪模式在死区内保留积分输出，P/D/本次积分增量清零，并保留真实求和比例项。
 *       未启用时死区输出仍为 0；增量式 Ki 为 0 时输出清零。应用层禁用电机堵转检测。
 */
#include "pid.h"
#include <stdbool.h>

static void f_Trapezoid_Intergral(PID_TypeDef *pid);
static void f_Integral_Limit(PID_TypeDef *pid);
static void f_Derivative_On_Measurement(PID_TypeDef *pid);
static void f_Changing_Integral_Rate(PID_TypeDef *pid);
static void f_Output_Filter(PID_TypeDef *pid);
static void f_Derivative_Filter(PID_TypeDef *pid);
static void f_Output_Limit(PID_TypeDef *pid);
static void f_Proportion_Limit(PID_TypeDef *pid);
static void f_PID_ErrorHandle(PID_TypeDef *pid);
static float Pid_CalculateCore(PID_TypeDef *pid, float measure, float target,
                               bool external_anti_windup, float lower, float upper);

static void f_PID_param_init(
    PID_TypeDef *pid,
    float max_out,
    float intergral_limit,
    float deadband,

    float kp,
    float Ki,
    float Kd,

    float Changing_Integral_A,
    float Changing_Integral_B,

    float output_filtering_coefficient,
    float derivative_filtering_coefficient,

    uint16_t improve)
{
    pid->DeadBand = deadband;
    pid->IntegralLimit = intergral_limit;
    pid->MaxOut = max_out;
    pid->MaxErr = max_out * 2;
    pid->Target = 0;

    pid->Kp = kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->ITerm = 0;

    pid->ScalarA = Changing_Integral_A;
    pid->ScalarB = Changing_Integral_B;

    pid->Output_Filtering_Coefficient = output_filtering_coefficient;

    pid->Derivative_Filtering_Coefficient = derivative_filtering_coefficient;

    pid->Improve = improve;

    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;

    pid->Output = 0;
}

/**
 * @brief 清零运行状态及历史，保留系数、限幅、滤波配置和函数指针。
 * @param[in,out] pid 需要复位的 PID 实例。
 * @pre 实例已初始化；调用时不得与同一实例的计算并发。
 */
static void f_PID_reset(PID_TypeDef *pid)
{
    pid->Target = 0.0f;
    pid->LastNoneZeroTarget = 0.0f;
    pid->Measure = 0.0f;
    pid->Last_Measure = 0.0f;
    pid->Err = 0.0f;
    pid->Last_Err = 0.0f;
    pid->L_Last_Err = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->ITerm = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->thistime = 0U;
    pid->lasttime = 0U;
    pid->dtime = 0U;
    pid->ERRORHandler.ERRORCount = 0U;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
}

/**
 * @brief 共用位置式计算流程，每次调用仅推进一次误差、微分及滤波历史。
 * @param[in] external_anti_windup 本次受限调用是否启用外部条件积分。
 * @param[in] lower 外部下界，单位与输出一致；未启用外部条件积分时不读取。
 * @param[in] upper 外部上界，单位与输出一致；未启用外部条件积分时不读取。
 * @note 只保存旧积分，因为内部积分限幅也会直接修改 Iout；P/D 和历史仍需保留。
 *       拒绝本次积分后仅重合成输出，不重算 P/D、不修改 Ki、不重复推进历史。
 */
static float Pid_CalculateCore(PID_TypeDef *pid, float measure, float target,
                               bool external_anti_windup, float lower, float upper)
{
    if(pid->Improve & ErrorHandle)
        f_PID_ErrorHandle(pid);

    if((pid->Improve & ErrorHandle) && pid->ERRORHandler.ERRORType != PID_ERROR_NONE)
    {
        pid->Output = 0.0f;
    }
    else
    {
        pid->Measure = measure;
        pid->Target = target;
        pid->Err = pid->Target - pid->Measure;

        if(ABS(pid->Err) > pid->DeadBand)
        {
            float iout_before = pid->Iout;

            pid->Pout = pid->Kp * pid->Err;
            pid->ITerm = pid->Ki * pid->Err;
            pid->Dout = pid->Kd * (pid->Err - pid->Last_Err);

            if(pid->Improve & Trapezoid_Intergral)
                f_Trapezoid_Intergral(pid);

            if(pid->Improve & ChangingIntegralRate)
                f_Changing_Integral_Rate(pid);

            if(pid->Improve & Integral_Limit)
                f_Integral_Limit(pid);

            if(pid->Improve & Derivative_On_Measurement)
                f_Derivative_On_Measurement(pid);

            if(pid->Improve & DerivativeFilter)
                f_Derivative_Filter(pid);

            pid->Iout += pid->ITerm;
            pid->Output = pid->Pout + pid->Iout + pid->Dout;

            if(pid->Improve & OutputFilter)
                f_Output_Filter(pid);

            f_Output_Limit(pid);

            if(external_anti_windup
               && ((pid->Output >= upper && pid->Iout > iout_before)
                   || (pid->Output <= lower && pid->Iout < iout_before)))
            {
                /* 拒绝同向积分，但动态积分范围缩小时不能恢复范围外的旧值。 */
                pid->Iout = iout_before;
                if(pid->Improve & Integral_Limit)
                {
                    if(pid->Iout > pid->IntegralLimit)
                        pid->Iout = pid->IntegralLimit;
                    else if(pid->Iout < -pid->IntegralLimit)
                        pid->Iout = -pid->IntegralLimit;
                }

                pid->ITerm = 0.0f;
                pid->Output = pid->Pout + pid->Iout + pid->Dout;
                if(pid->Improve & OutputFilter)
                    f_Output_Filter(pid);

                f_Output_Limit(pid);
            }

            /* 跟踪回算需要真实参与求和的 Pout，不能在输出合成后只改写比例项。 */
            if(!(pid->Improve & PID_OUTPUT_TRACKING))
                f_Proportion_Limit(pid);
        }
        else if(pid->Improve & PID_OUTPUT_TRACKING)
        {
            pid->Pout = 0.0f;
            pid->Dout = 0.0f;
            pid->ITerm = 0.0f;
            pid->Output = pid->Iout;
            f_Output_Limit(pid);
        }
        else
        {
            pid->Output = 0.0f;
        }

        pid->Last_Measure = pid->Measure;
        pid->Last_Output = pid->Output;
        pid->Last_Dout = pid->Dout;
        pid->Last_Err = pid->Err;
    }

    return pid->Output;
}

float PID_Calculate(PID_TypeDef *pid, float measure, float target)
{
    return Pid_CalculateCore(pid, measure, target, false, 0.0f, 0.0f);
}

/**
 * @brief 单次执行位置式计算，再按外部边界限制输出。
 * @note 外部抗饱和仅由本接口和选项共同启用，普通接口仍保持原计算行为。
 */
float PID_CalculateConstrained(PID_TypeDef *pid, float measure, float target,
                               float lower, float upper)
{
    Pid_CalculateCore(pid, measure, target,
                      (pid->Improve & PID_EXTERNAL_ANTI_WINDUP) != 0U, lower, upper);

    if(pid->Output < lower)
    {
        if(pid->Improve & PID_OUTPUT_TRACKING)
            PID_TrackOutput(pid, lower);
        else
            pid->Output = lower;
    }
    else if(pid->Output > upper)
    {
        if(pid->Improve & PID_OUTPUT_TRACKING)
            PID_TrackOutput(pid, upper);
        else
            pid->Output = upper;
    }

    return pid->Output;
}

/**
 * @brief 按真实 P/D 贡献回算积分，不重复更新同次采样的误差或微分历史。
 * @note 积分范围不足时保留饱和事实；Output 仍记录外部实际采用值，不冒充完全无扰。
 */
void PID_TrackOutput(PID_TypeDef *pid, float applied_output)
{
    pid->Iout = applied_output - pid->Pout - pid->Dout;
    if(pid->Iout > pid->IntegralLimit)
        pid->Iout = pid->IntegralLimit;
    else if(pid->Iout < -pid->IntegralLimit)
        pid->Iout = -pid->IntegralLimit;

    pid->ITerm = 0.0f;
    pid->Output = applied_output;
    pid->Last_Output = applied_output;
}

float PID_Calculate_Increase(PID_TypeDef *pid, float measure, float target)
{
    pid->Measure = measure;
    pid->Target = target;

    pid->L_Last_Err = pid->Last_Err;
    pid->Last_Err = pid->Err;
    pid->Err = pid->Target - pid->Measure;

    pid->Pout = pid->Kp * (pid->Err - pid->Last_Err);
    pid->Iout = pid->Ki * pid->Err;
    pid->Dout = pid->Kd * (pid->Err - 2 * pid->Last_Err + pid->L_Last_Err);

    pid->ITerm = pid->Pout + pid->Iout + pid->Dout;

    OUTPUT_LIMIT(pid->ITerm, pid->MaxOut, -pid->MaxOut);

    pid->Output += pid->ITerm;

    if(pid->Ki == 0)
        pid->Output = 0;

    f_Output_Limit(pid);

    pid->Last_Measure = measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;

    return pid->Output;
}

float PID_Cascade_Calculate(PID_TypeDef *pos_pid, PID_TypeDef *vel_pid, float pos_measure, float vel_measure, float pos_target)
{
    PID_Calculate(pos_pid, pos_measure, pos_target);
    PID_Calculate(vel_pid, vel_measure, pos_pid->Output);

    return vel_pid->Output;
}

float PID_Cascade_Calculate_Inner_Increase(PID_TypeDef *pos_pid, PID_TypeDef *vel_pid, float pos_measure, float vel_measure, float pos_target)
{
    PID_Calculate(pos_pid, pos_measure, pos_target);
    PID_Calculate_Increase(vel_pid, vel_measure, pos_pid->Output);

    return vel_pid->Output;
}

static void f_Trapezoid_Intergral(PID_TypeDef *pid)
{
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2);
}

static void f_Changing_Integral_Rate(PID_TypeDef *pid)
{
    if(pid->Err * pid->Iout > 0)
    {

        if(ABS(pid->Err) <= pid->ScalarB)
            return;
        if(ABS(pid->Err) <= (pid->ScalarA + pid->ScalarB))
            pid->ITerm *= (pid->ScalarA - ABS(pid->Err) + pid->ScalarB) / pid->ScalarA;
        else
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PID_TypeDef *pid)
{
    float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if(ABS(temp_Output) > pid->MaxOut)
    {
        if(pid->Err * pid->Iout > 0)
        {

            pid->ITerm = 0;
        }
    }

    if(temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if(temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

static void f_Derivative_On_Measurement(PID_TypeDef *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure);
}

static void f_Derivative_Filter(PID_TypeDef *pid)
{
    pid->Dout = pid->Dout * pid->Derivative_Filtering_Coefficient +
    pid->Last_Dout * (1 - pid->Derivative_Filtering_Coefficient);
}

static void f_Output_Filter(PID_TypeDef *pid)
{
    pid->Output = pid->Output * pid->Output_Filtering_Coefficient +
    pid->Last_Output * (1 - pid->Output_Filtering_Coefficient);
}

static void f_Output_Limit(PID_TypeDef *pid)
{
    if(pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if(pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

static void f_Proportion_Limit(PID_TypeDef *pid)
{

    if(pid->Pout > pid->MaxOut)
    {
        pid->Pout = pid->MaxOut;
    }
    if(pid->Pout < -(pid->MaxOut))
    {
        pid->Pout = -(pid->MaxOut);
    }
}

static void f_PID_ErrorHandle(PID_TypeDef *pid)
{

    if(pid->Output < pid->MaxOut * 0.01f)
        return;

    if((ABS(pid->Target - pid->Measure) / pid->Target) > 0.9f)
    {

        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if(pid->ERRORHandler.ERRORCount > 1000)
    {

        pid->ERRORHandler.ERRORType = Motor_Blocked;
    }
}

void PID_Init(PID_TypeDef *pid, float max_out, float intergral_limit, float deadband,

    float kp, float Ki, float Kd,

    float A, float B,

    float output_filtering_coefficient, float derivative_filtering_coefficient, uint16_t improve)
{
    pid->PID_param_init = f_PID_param_init;
    pid->PID_reset = f_PID_reset;
    pid->PID_param_init(pid, max_out, intergral_limit, deadband,
        kp, Ki, Kd, A, B, output_filtering_coefficient, derivative_filtering_coefficient, improve);
}

float FeedforwardController(FFC_t *vFFC,float a,float b)
{

    float result;

    result=a*(vFFC->rin-vFFC->lastRin)+b*(vFFC->rin-2*vFFC->lastRin+vFFC->perrRin);

    vFFC->perrRin= vFFC->lastRin;

    vFFC->lastRin= vFFC->rin;

    return result;

}

void lag_compensator_init(LagCompensator *lc, float lag_filter)
{
    lc->output = 0.0;
    lc->lag_filter = lag_filter;
}

float lag_compensator_compute(LagCompensator *lc, float input)
{
    lc->output = lc->lag_filter * lc->output + (1 - lc->lag_filter) * input;
    return lc->output;
}
