/**
 * @file pid.h
 * @brief Taurus-arm 指定 PID 库的本工程副本，保留原计算及公开类型。
 * @details 来源 Hongxi Wong V1.0.6；仅整理编码、排版、注释与私有声明位置。
 * @note 系数按每次调用定义，不自动处理 dt。初始化前由调用者清零结构体；
 *       原位置式死区分支输出 0（死区为 0 时误差恰为 0 亦如此），增量式
 *       Ki 为 0 时输出清零。本轮保留这些行为；应用层禁用电机堵转检测。
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

    uint8_t Improve;

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
        uint8_t improve);

    void (*PID_reset)(
        struct _PID_TypeDef *pid,
        float Kp,
        float ki,
        float kd);
} PID_TypeDef;

/** @brief 原库增量式计算；measure/target 同单位，返回累计输出。 */
float PID_Calculate_Increase(PID_TypeDef *pid, float measure, float target);
/** @brief 设置原库参数与函数指针；调用者先清零完整实例，系数按每次调用定义。 */
void PID_Init(PID_TypeDef *pid, float max_out, float intergral_limit, float deadband,

    float kp, float Ki, float Kd,

    float A, float B,

    float output_filtering_coefficient, float derivative_filtering_coefficient, uint8_t improve);
/** @brief 原库位置式计算；非空已初始化实例，measure/target 同单位；返回限幅输出。 */
float PID_Calculate(PID_TypeDef *pid, float measure, float target);
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
