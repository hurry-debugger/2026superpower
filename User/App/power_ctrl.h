/** @file power_ctrl.h
 * @brief 超电四环控制、三状态保护及调试接口。
 */
#ifndef SUPERPOWER_POWER_CTRL_H
#define SUPERPOWER_POWER_CTRL_H

#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief 用户指定状态名；编译测试模式与运行状态彼此独立。 */
typedef enum
{
    PROTECT_MODE = 0,
    RESTART_MODE,
    CTRL_MODE
} PowerCtrlState_e;

/** @brief 当前有效错误位；计数故障与毫秒超时分别处理。 */
typedef enum
{
    POWER_CTRL_ERROR_NONE = 0U,
    POWER_CTRL_ERROR_CAP_OVERVOLTAGE = 1U << 0,
    POWER_CTRL_ERROR_CHASSIS_OVERVOLTAGE = 1U << 1,
    POWER_CTRL_ERROR_BAT_UNDERVOLTAGE = 1U << 2,
    POWER_CTRL_ERROR_REFEREE = 1U << 3,
    POWER_CTRL_ERROR_CALIBRATION = 1U << 4,
    POWER_CTRL_ERROR_SAMPLE = 1U << 5,
    POWER_CTRL_ERROR_CONFIG = 1U << 6,
    POWER_CTRL_ERROR_NUMERIC = 1U << 7,
    POWER_CTRL_ERROR_REMOTE = 1U << 8,
    POWER_CTRL_ERROR_TIMING = 1U << 9
} PowerCtrlError_e;

/** @brief App 唯一写入的控制状态，其他文件按约定只读，不可重入访问。 */
typedef struct
{
    PowerCtrlState_e state;
    uint32_t errors;
    uint32_t restart_elapsed_ms;
    uint32_t cap_overvoltage_cnt;
    uint32_t chassis_overvoltage_cnt;
    uint32_t bat_undervoltage_cnt;
    bool output_enabled;
    bool voltage_competing;
    bool open_loop_enable;
    float test_power_w; /**< 无裁判目标 W，可为负；写入使用 SetTestPower。 */
    float open_loop_duty; /**< 无量纲，写入使用 SetOpenLoopDuty。 */
    float power_feedback_w; /**< chassis 端 V*I，带符号 W。 */
    float power_target_w; /**< 限制后的目标 W。 */
    float current_limit_a; /**< 15*Vcap/Vchassis，单位 A。 */
    float current_target_a; /**< 竞争后带符号电流 A。 */
    float generalized_duty; /**< 电压比前馈加电流 PID 修正，无量纲。 */
    PID_TypeDef energy_pid;
    PID_TypeDef power_pid;
    PID_TypeDef voltage_pid;
    PID_TypeDef current_pid;
} PowerCtrl_t;

extern PowerCtrl_t g_power_ctrl;

/**
 * @brief 关闭输出、清除状态并以数值常量初始化四个 PID，上电进入 PROTECT。
 * @pre MX_HRTIM1_Init 已完成；唯一调用上下文，无并发外设写入。
 * @note 不初始化采样或裁判存储，不启动计数器，不修改外设配置。
 */
void PowerCtrl_Init(void);
/**
 * @brief 依次切换状态和计算控制；从 BSP 全局结构体读取反馈。
 * @pre 已 Init；唯一固定周期调用者，周期为 POWER_CTRL_PERIOD_US。
 *      同一上下文先完成 BSP 整组发布，再调用 Task，无异步写入。
 *      HAL_GetTick 保持单调毫秒时基；计数器运行、预装载等由集成者配置。
 * @note Master 内先处理裁判和稳定采样，再调用本接口；PWM 装载等待按毫秒向上取整。
 */
void PowerCtrl_Task(void);
/**
 * @brief Master 发现错过下一次 MREP 时立即保护，锁存至重新 Init。
 * @pre 已 Init；仅 Master 同一串行上下文调用，不在其他中断重入。
 * @note 立即关闭输出并复位环路，不通过恢复冷却自动清除此故障。
 */
void PowerCtrl_NotifyTimingFault(void);
/** @brief 串行调试接口：设置无裁判目标 W，允许负值；无效数值触发保护。 */
void PowerCtrl_SetTestPower(float power_w);
/** @brief 串行调试接口：设置开环无量纲 g，必须位于 BSP 支持范围内。 */
void PowerCtrl_SetOpenLoopDuty(float generalized_duty);
/** @brief 开环测试允许开关，初始 false；关闭立即关断并回到 PROTECT。 */
void PowerCtrl_SetOpenLoopEnable(bool enable);

#endif /* SUPERPOWER_POWER_CTRL_H */
