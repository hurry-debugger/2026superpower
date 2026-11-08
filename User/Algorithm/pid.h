/**
 * @file pid.h
 * @brief Taurus-arm PID 库的本工程副本，提供可选的外部限幅抗积分饱和。
 * @details 来源 Hongxi Wong V1.0.6；外部限幅抗饱和和位置式输出跟踪均由 Improve 显式启用。
 *          未启用跟踪的实例保留原算法；复位仅清运行状态并保留参数。
 * @note 系数按每次调用定义，不自动处理 dt。初始化前由调用者清零结构体；
 *       跟踪模式在死区内保留积分输出，P/D/本次积分增量清零，并保留真实求和比例项。
 *       未启用时死区输出仍为 0；增量式 Ki 为 0 时输出清零。应用层禁用电机堵转检测。
 */
#ifndef _PID_H
#define _PID_H

#include "stdint.h"

#ifndef OUTPUT_LIMIT
#define OUTPUT_LIMIT(output,max,min) \
( (output) <= (max) && (output)>=(min)? output: \
( (output) > (max)? (output = max):(output = min)) )
#endif

#ifndef ABS
#define ABS(x) ((x > 0) ? x : -(x))
#endif

typedef enum pid_Improvement_e
{
    NONE = 0X00,
    Integral_Limit = 0x01,
    Derivative_On_Measurement = 0x02,
    Trapezoid_Intergral = 0x04,
    Proportional_On_Measurement = 0x08,
    OutputFilter = 0x10,
    ChangingIntegralRate = 0x20,
    DerivativeFilter = 0x40,
    ErrorHandle = 0x80,
    PID_EXTERNAL_ANTI_WINDUP = 0x100U, /**< 外部限幅时撤销继续推动饱和的本次积分。 */
    PID_OUTPUT_TRACKING = 0x200U, /**< 位置式输出跟踪；死区保留积分，Pout 保留真实贡献。 */
} PID_Improvement_e;

typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    Motor_Blocked = 0x01U
} ErrorType_e;

typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

typedef struct _PID_TypeDef
{
    float Target;
    float LastNoneZeroTarget;
    float Kp;
    float Ki;
    float Kd;

    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float L_Last_Err;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float MaxOut;
    float IntegralLimit;
    float DeadBand;
    float ControlPeriod;
    float MaxErr;
    float ScalarA;
    float ScalarB;
    float Output_Filtering_Coefficient;
    float Derivative_Filtering_Coefficient;

    uint32_t thistime;
    uint32_t lasttime;
    uint8_t dtime;

    uint16_t Improve;

    PID_ErrorHandler_t ERRORHandler;

    void (*PID_param_init)(
        struct _PID_TypeDef *pid,
        float maxOut,
        float integralLimit,
        float deadband,
        float Kp,
        float ki,
        float kd,
        float A,
        float B,
        float output_filtering_coefficient,
        float derivative_filtering_coefficient,
        uint16_t improve);

    /**
     * @brief 清零运行状态及历史，保留全部参数和函数指针。
     * @param[in,out] pid 已初始化的 PID 实例。
     * @pre 不得与同一实例的计算并发。
     */
    void (*PID_reset)(struct _PID_TypeDef *pid);
} PID_TypeDef;

/** @brief 原库增量式计算；measure/target 同单位，返回累计输出。 */
float PID_Calculate_Increase(PID_TypeDef *pid, float measure, float target);
/** @brief 设置原库参数与函数指针；调用者先清零完整实例，系数按每次调用定义。 */
void PID_Init(PID_TypeDef *pid, float max_out, float intergral_limit, float deadband,

    float kp, float Ki, float Kd,

    float A, float B,

    float output_filtering_coefficient, float derivative_filtering_coefficient, uint16_t improve);
/** @brief 原库位置式计算；非空已初始化实例，measure/target 同单位；返回限幅输出。 */
float PID_Calculate(PID_TypeDef *pid, float measure, float target);
/**
 * @brief 执行位置式 PID，并按外部上下限约束本次输出。
 * @param[in,out] pid 已初始化的实例；Improve 含 PID_EXTERNAL_ANTI_WINDUP 时启用抗饱和。
 * @param[in] measure 本次反馈，与 target 同单位。
 * @param[in] target 本次目标，与 measure 同单位。
 * @param[in] lower 外部输出下限，单位与 PID 输出一致。
 * @param[in] upper 外部输出上限，单位与 PID 输出一致。
 * @return 外部限幅后的输出，同时写入 pid->Output。
 * @pre pid 非空，lower <= upper；不得与同一实例的计算或复位并发。
 * @note 启用时在单次计算内拒绝继续推动饱和的本次积分，保留既有积分及当前积分限幅；
 *       不修改 Ki，不复制完整状态，P/D 和采样历史每次调用只更新一次。
 *       跟踪模式在外部截断后回算积分并对齐 Last_Output，动态边界缩小时不保留旧输出历史；
 *       未启用跟踪时沿用原限幅及 Last_Output 行为。跟踪实例不得启用 OutputFilter。
 */
float PID_CalculateConstrained(PID_TypeDef *pid, float measure, float target,
                               float lower, float upper);
/**
 * @brief 让位置式 PID 跟踪实际采用的输出，保留本周期误差及微分历史。
 * @param[in,out] pid 本周期已计算的 PID_OUTPUT_TRACKING 实例。
 * @param[in] applied_output 实际采用的输出，单位与 pid->Output 一致，须在应用输出范围内。
 * @pre 实例非空且已初始化，OutputFilter 关闭；不得与计算或复位并发。
 * @note 用实际输出减去本周期 Pout/Dout 回算积分，并限制到 IntegralLimit；
 *       Output/Last_Output 记录实际采用值，ITerm 清零。回算受积分范围限制，
 *       不保证下次候选完全重现跟踪值；本接口不更新测量或误差，须先执行本周期计算。
 */
void PID_TrackOutput(PID_TypeDef *pid, float applied_output);
/** @brief 外环及内环均位置式；外环输出作为内环目标，单位由应用约定。 */
float PID_Cascade_Calculate(PID_TypeDef *pos_pid, PID_TypeDef *vel_pid, float pos_measure, float vel_measure, float pos_target);
/** @brief 外环位置式、内环增量式；指针非空且均已初始化。 */
float PID_Cascade_Calculate_Inner_Increase(PID_TypeDef *pos_pid, PID_TypeDef *vel_pid, float pos_measure, float vel_measure, float pos_target);

typedef struct
{
    float output;
    float lag_filter;
} LagCompensator;

typedef struct
{
    float rin;
    float lastRin;
    float perrRin;
}FFC_t;

extern FFC_t yaw_ffc;

/** @brief 按原库两级输入历史计算前馈并更新历史；系数单位由应用定义。 */
float FeedforwardController(FFC_t *vFFC,float a,float b);

typedef struct
{
    float setpoint;
    float kp;
    float ki;
    float kd;
    float lasterror;
    float preerror;
    float result;
    float output;
    float maximum;
    float minimum;
    float errorabsmax;
    float errorabsmid;
    float errorabsmin;
}EXPERTPID;

/** @brief 初始化原库滞后补偿器，参数为离散滤波系数。 */
void lag_compensator_init(LagCompensator *lc, float lag_filter);
/** @brief 执行原库滞后补偿，返回本次输出；非空指针由调用者保证。 */
float lag_compensator_compute(LagCompensator *lc, float input);

#endif
