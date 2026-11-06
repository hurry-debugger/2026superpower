/** @file power_ctrl.h
 * @brief 固定开环波形、超电四环闭环控制及两状态保护。
 */
#ifndef SUPERPOWER_POWER_CTRL_H
#define SUPERPOWER_POWER_CTRL_H

#include "pid.h"
#include <stdint.h>

/** @brief 用户指定状态名；编译测试模式与运行状态彼此独立。 */
typedef enum
{
    PROTECT_MODE = 0,
    CTRL_MODE = 2
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
        uint32_t cap_overvoltage : 1; /**< 电容过压：电压超过阈值并达到连续次数。 */
        uint32_t chassis_overvoltage : 1; /**< 底盘过压：电压超过阈值，恢复采用迟滞判断。 */
        uint32_t bat_undervoltage : 1; /**< 电池欠压：电压低于阈值并达到连续次数。 */
        uint32_t referee : 1; /**< 裁判数据异常：闭环模式任一所需报文无效或接收超时。 */
        uint32_t calibration : 1; /**< 校准异常：板卡识别失败或任一通道校准未通过。 */
        uint32_t sample : 1; /**< 采样数值异常：五路反馈中存在 NaN 或无穷值，不含停更检测。 */
        uint32_t config : 1; /**< 控制配置异常：闭环参数/PWM 未就绪或运行状态非法。 */
        uint32_t numeric : 1; /**< 运算条件异常：反馈非有限、电压非正或输出非有限。 */
        uint32_t remote : 1; /**< 远程保护请求：CAN 底盘模式命令为 1。 */
        uint32_t short_circuit : 1; /**< 底盘低压且电流越限，单次触发锁存至 MCU 复位。 */
        uint32_t reserved : 22; /**< 保留，保持为 0。 */
    } bits; /**< 按具体错误类型读写的位域视图，与 all 共用存储。 */
} PowerCtrlError_t;

/**
 * @brief App 唯一写入的控制状态，其他文件按约定只读，不可重入访问。
 * @note 功率反馈、目标、限流和占空比为控制计算记录，保护期间不代表全都实时更新。
 *       竞争限幅后的电流目标直接观察 power_pid.Output，不另存同值成员。
 *       开环仅更新 state/errors，其余控制记录保持初始化值，不表示 PWM 实际占空比。
 */
typedef struct
{
    PowerCtrlState_e state;
    PowerCtrlError_t errors; /**< 本周期错误及短路锁存，支持位域访问及整数汇总。 */
    uint32_t cap_overvoltage_cnt;
    uint32_t chassis_overvoltage_cnt;
    uint32_t bat_undervoltage_cnt;
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
 * @brief 关闭输出、清除状态，上电进入 PROTECT；仅闭环初始化四个 PID。
 * @pre MX_HRTIM1_Init 已完成；唯一调用上下文，无并发外设写入。
 * @note 仅启动时调用，清除短路锁存；不提供运行期故障清除接口。
 *       不初始化采样或裁判存储，不启动计数器，不修改外设配置。
 */
void PowerCtrl_Init(void);
/**
 * @brief 每 25 kHz 周期更新错误并管理输出；闭环计算四环，开环保持固定 50% 波形。
 * @pre 必须先调用 PowerCtrl_Init；唯一固定周期调用者，每次 ADC1 TC 先更新五路反馈。
 *      控制周期为 POWER_CTRL_PERIOD_US，同一上下文先完成 BSP 处理，再调用 Task。
 *      CubeMX 已将 A/B 的 CMP1/CMP2 配置为周期的 1/4、3/4，保留互补死区及半周期错相。
 *      首次 ADC1 TC 前，计数器及比较值装载已运行至少一个完整 PWM 周期。
 *      闭环 HAL_GetTick 保持单调毫秒时基；裁判仅由低优先级串口上下文原子发布。
 * @note 开环必须外部断开驱动供电或信号，软件不能保证有 PWM 时 MOS 不动作。
 *       开环首次 Task 自动开启；仅远程保护、PWM 未就绪或非法状态阻止输出。
 *       开环不运行 PID、不写比较值，不检查校准、采样、电压、参数或裁判门槛；解除远程保护后恢复。
 *       闭环在 switch 前统一计算并检查前馈；PROTECT 每周期关断并预置有效平衡点。
 *       首次全部运行条件有效且完成预置后开始计时，毫秒差值达到 500 即转 CTRL。
 *       普通故障取消等待；短路单次锁存至 MCU 复位，不自动恢复。
 *       闭环 PROTECT 转 CTRL 时复位四环及目标记录，使用已装载平衡点单次开启输出，随后计算控制。
 *       持续 CTRL 不重复开启；运算异常本周期立即关断；不额外增加 tick 或装载等待阶段。
 *       两模式均不检测采样停更或执行超期；输出及控制状态只能由 ADC1 DMA 上下文管理。
 */
void PowerCtrl_Task(void);

#endif /* SUPERPOWER_POWER_CTRL_H */
