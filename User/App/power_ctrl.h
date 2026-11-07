/** @file power_ctrl.h
 * @brief 25 kHz 过流/控制、1 kHz 普通保护及自动恢复。
 */
#ifndef SUPERPOWER_POWER_CTRL_H
#define SUPERPOWER_POWER_CTRL_H

#include "pid.h"
#include <stdint.h>

/** @brief 用户指定状态名；编译测试模式与运行状态彼此独立。 */
typedef enum
{
    PROTECT_MODE = 0,
    CTRL_MODE = 1
} PowerCtrlState_e;

/**
 * @brief 电源控制错误标志；各成员 1 表示错误有效，0 表示无该错误。
 * @note all 与 bits 共享存储；all 只用于内部错误汇总，不直接序列化。
 *       使用目标编译器位域布局，不规定各错误的整数位序；保留成员始终为 0。
 */
typedef union
{
    uint32_t all; /**< 完整错误字；0 表示无错误，非 0 表示存在错误。 */
    struct
    {
        uint32_t cap_voltage : 1; /**< 电容电压不满足 (0,31] V，含无效数值。 */
        uint32_t chassis_voltage : 1; /**< 底盘电压异常或分级过压累计达到门槛。 */
        uint32_t bat_voltage : 1; /**< 电池电压非有限或不足 15 V，无物理电压上限。 */
        uint32_t referee : 1; /**< 共用接收时间戳超时，不代表完整报文有效性检查。 */
        uint32_t calibration : 1; /**< 启动时全通道校准掩码不完整，运行期不变。 */
        uint32_t power_off : 1; /**< 裁判关闭底盘输出，不重复判断电池电压。 */
        uint32_t short_circuit : 1; /**< 底盘电流绝对值超过 30 A，锁存至 MCU 复位。 */
        uint32_t reserved : 25; /**< 保留，保持为零。 */
    } bits; /**< 按具体错误类型读写的位域视图，与 all 共用存储。 */
} PowerCtrlError_t;

/**
 * @brief ADC 管理输出和 PID，Master 发布普通错误及保护状态。
 * @note volatile 标明共享访问，快照及错误字发布仍需短临界区保证一致性。
 *       功率、目标、限流及占空比在保护期间不全部实时更新。
 *       竞争限幅后的电流目标直接观察 power_pid.Output，不另存同值成员。
 *       开环回调只设置 state；反馈、错误及其余控制记录保持初始化值。
 */
typedef struct
{
    volatile PowerCtrlState_e state;
    volatile PowerCtrlError_t errors; /**< 普通错误重算，校准汇总与短路锁存保留。 */
    volatile uint32_t errclearcnt; /**< ADC 累计恢复周期；错误发布或进入 CTRL 时清零。 */
    uint32_t chassis_overvoltage_cnt; /**< 仅 1 kHz 更新的加权累计，范围 0～300。 */
    float power_feedback_w; /**< chassis 端 V*I，带符号 W。 */
    float power_target_w; /**< 限制后的目标 W。 */
    float current_limit_a; /**< 15*Vcap/Vchassis，单位 A。 */
    float generalized_duty; /**< 无量纲；先存本周期前馈，CTRL 最后保存闭环输出，开环不使用。 */
    PID_TypeDef energy_pid;
    PID_TypeDef power_pid;
    PID_TypeDef voltage_pid; /**< Output 为允许的充电电流 A，未竞争时为电流上限。 */
    PID_TypeDef current_pid;
} PowerCtrl_t;

extern PowerCtrl_t g_power_ctrl;

/**
 * @brief 共用启动初始化：关闭输出、清零状态、进入 PROTECT 并初始化四环。
 * @pre MX_HRTIM1_Init 已完成，计数器和控制中断尚未启动；仅调用一次。
 * @note 仅此入口清除短路锁存；闭环调度随后设置校准汇总并预先检查普通错误。
 *       不初始化采样或裁判存储，不启动计数器，不修改外设配置。
 */
void PowerCtrl_Init(void);

/**
 * @brief 1 kHz 更新普通故障、分级过压累计及下电条件。
 * @pre 启动时已设置 calibration 汇总位；运行期只由 Master 1 kHz 调用。
 *      允许在 ADC 启动前预调用一次，零初始电压使首次周期检查前保持保护。
 * @note 快照只复制三个电压和错误字；区外计算，发布时保留最新短路位。
 *       有错误即发布 PROTECT 并清零恢复计数，下一次 ADC 控制关断 PWM。
 *       无错误不直接开启或复位 PID；校准汇总和短路锁存不随普通故障解除。
 *       裁判时间戳沿用现有接收代码，不提供新的报文有效性判据。
 */
void PowerCtrl_UpdateErrors(void);

/**
 * @brief 25 kHz 检查底盘过流，执行状态切换及四环控制。
 * @pre ADC1 TC 先完成五路反馈处理；控制周期为 POWER_CTRL_PERIOD_US。
 *      ADC1 DMA 优先级 0、Master 为 1；计数器及预装载已运行至少一个 PWM 周期。
 * @note 短路只判断底盘电流绝对值，命中置位并转 PROTECT，同周期关断，禁止自动重启。
 *       普通错误由 Master 更新，不在此重复检测；PROTECT 每周期关断并预置前馈。
 *       第 12501 个连续正常控制周期复位四环、清零计数并单次开启；持续 CTRL 不重复开启。
 *       PWM、比较值写入及 PID 复位只由 ADC 上下文执行。
 *       开环不调用此入口；不增加采样停更、执行超期或 CAN 模式控制。
 */
void PowerCtrl_Task(void);

#endif /* SUPERPOWER_POWER_CTRL_H */
