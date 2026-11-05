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
    POWER_CTRL_ERROR_REMOTE = 1U << 8
} PowerCtrlError_e;

/**
 * @brief App 唯一写入的控制状态，其他文件按约定只读，不可重入访问。
 * @note 功率反馈、目标、限流和占空比为控制计算记录，保护期间不代表全都实时更新。
 *       竞争限幅后的电流目标直接观察 power_pid.Output，不另存同值成员。
 */
typedef struct
{
    PowerCtrlState_e state;
    uint32_t errors;
    uint32_t cap_overvoltage_cnt;
    uint32_t chassis_overvoltage_cnt;
    uint32_t bat_undervoltage_cnt;
    bool open_loop_enable;
    float test_power_w; /**< 无裁判目标 W，可为负；写入使用 SetTestPower。 */
    float open_loop_duty; /**< 无量纲，写入使用 SetOpenLoopDuty。 */
    float power_feedback_w; /**< chassis 端 V*I，带符号 W。 */
    float power_target_w; /**< 限制后的目标 W。 */
    float current_limit_a; /**< 15*Vcap/Vchassis，单位 A。 */
    float generalized_duty; /**< 电压比前馈加电流 PID 修正，无量纲。 */
    PID_TypeDef energy_pid;
    PID_TypeDef power_pid;
    PID_TypeDef voltage_pid; /**< Output 为允许的充电电流 A，未竞争时为电流上限。 */
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
 * @brief 每 25 kHz 周期依次累计与判断错误、切换状态、计算占空比并执行输出。
 * @pre 必须先调用 PowerCtrl_Init；不使用初始化标志，唯一固定周期调用者。
 *      调用周期为 POWER_CTRL_PERIOD_US，从 BSP 全局结构体读取反馈。
 *      初始化时 BSP 反馈无效，随后每次 ADC1 TC 先更新反馈。
 *      同一上下文先完成 BSP 处理，再调用 Task，无异步反馈写入。
 *      HAL_GetTick 保持单调毫秒时基；计数器运行、预装载等由集成者配置。
 * @note 仅 ADC1 DMA 全满回调在 Process 后调用；裁判由串口回调发布。
 *       不检测采样停更或执行超期；调试接口须与控制串行调用。
 *       状态切换只修改 state，DutyCalcu 按状态转换执行启停；正常闭环单独封装。
 *       Init 先关断输出，首次进入 CTRL 才开启，进入保护先关断再复位环路。
 *       恢复等待保持 500 ms，PWM 装载等待为周期向上取整后再多等一个毫秒 tick。
 */
void PowerCtrl_Task(void);
/** @brief 串行调试接口：设置无裁判目标 W，允许负值；无效数值触发保护。 */
void PowerCtrl_SetTestPower(float power_w);
/** @brief 串行调试接口：设置开环无量纲 g，必须位于 BSP 支持范围内。 */
void PowerCtrl_SetOpenLoopDuty(float generalized_duty);
/**
 * @brief 开环测试允许开关，初始 false；开环模式下关闭立即关断并回到 PROTECT。
 * @pre 必须先调用 PowerCtrl_Init；与控制入口串行调用。
 */
void PowerCtrl_SetOpenLoopEnable(bool enable);

#endif /* SUPERPOWER_POWER_CTRL_H */
