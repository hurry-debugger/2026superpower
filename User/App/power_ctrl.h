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
 *       未校准提示保存在板卡信息中，不属于控制错误，不单独阻止闭环运行。
 */
typedef union
{
    uint32_t all; /**< 完整错误字；0 表示无错误，非 0 表示存在错误。 */
    struct
    {
        uint32_t cap_voltage : 1; /**< 电容电压不满足 (0, POWER_CTRL_OVERVOLTAGE_V]，当前上限 29 V，含无效数值。 */
        uint32_t chassis_voltage : 1; /**< 底盘电压异常或分级过压累计达到门槛。 */
        uint32_t bat_voltage : 1; /**< 电池电压不在 [15, ADC_V_BAT_MAX_V]，含无效数值。 */
        uint32_t referee : 1; /**< 共用接收时间戳超时，不代表完整报文有效性检查。 */
        uint32_t power_off : 1; /**< 裁判关闭底盘输出，不重复判断电池电压。 */
        uint32_t short_circuit : 1; /**< 底盘电流绝对值超过 30 A，锁存至 MCU 复位。 */
        uint32_t reserved : 26; /**< 保留，保持为零。 */
    } bits; /**< 按具体错误类型读写的位域视图，与 all 共用存储。 */
} PowerCtrlError_t;

/**
 * @brief ADC 管理输出、状态和 PID，Master 只发布普通错误字。
 * @note volatile 标明共享访问，快照及错误字发布仍需短临界区保证一致性。
 *       A_power_fdb 与 total_power_fbd 分别保存同周期 P_A 和 P_in，CTRL 更新 A 端目标；
 *       power_target_w 只保存目标功率，PROTECT 期间保留初值或上次目标，不用于控制。
 *       竞争限幅后的电流目标直接观察 power_pid.Output，不另存同值成员。
 *       电流环启用积分限幅和外部条件积分，其余三环保持 NONE；不启用滤波或输出跟踪。
 *       开环不调用控制入口，测量与控制记录保持初值；Master 仍可更新错误字。
 */
typedef struct
{
    volatile PowerCtrlState_e state;
    volatile PowerCtrlError_t errors; /**< 普通错误重算，短路锁存保留；不含未校准提示。 */
    volatile uint32_t errclearcnt; /**< ADC 累计恢复周期；保护条件成立或进入 CTRL 时清零。 */
    uint32_t chassis_overvoltage_cnt; /**< 仅 1 kHz 更新的加权累计，范围 0～300。 */
    float A_power_fdb; /**< A 端 V_CHASSIS*I_CHASSIS，单位 W；正充电、负放电。 */
    float total_power_fbd; /**< A 端功率与底盘用电功率之和的综合反馈，单位 W；由 V_BAT*I_BAT 计算。 */
    float power_target_w; /**< A 端目标功率 P_lim-P_in+P_A-u_E，单位 W；仅 CTRL 更新。 */
    float current_upper_limit_a; /**< A 端正向目标上界 A；B 端充电参考上限乘原始前馈。 */
    float current_lower_limit_a; /**< A 端负向目标下界 A；低于最大电压的 20%（当前 5.8 V）为零，禁止主动请求放电。 */
    float generalized_duty; /**< 有效电压时先存原始前馈，CTRL 最后存输出；无效时不覆盖。 */
    PID_TypeDef energy_pid; /**< 45 J 能量误差生成修正 u_E，W；每个控制周期计算。 */
    PID_TypeDef power_pid; /**< A 端目标与反馈之差等价于入口功率误差；输出目标电流 A。 */
    PID_TypeDef voltage_pid; /**< 无量纲占空比修正；每周期计算，严格高于最大电压的 85%（当前 24.65 V）参与竞争。 */
    PID_TypeDef current_pid; /**< A 端电流误差生成无量纲广义占空比修正。 */
} PowerCtrl_t;

extern PowerCtrl_t g_power_ctrl;

/**
 * @brief 共用启动初始化：关闭输出、清零状态、进入 PROTECT 并初始化四环。
 * @pre MX_HRTIM1_Init 已完成，计数器和控制中断尚未启动；仅调用一次。
 * @note 未校准提示由板卡信息独立记录；本入口不检查校准状态，不预查普通错误。
 *       不初始化采样或裁判存储，不启动计数器，不修改外设配置。
 */
void PowerCtrl_Init(void);

/**
 * @brief 1 kHz 更新普通故障、分级过压累计及下电条件。
 * @pre 控制已初始化；运行期只由 Master 1 kHz 调用，不要求通道均已校准。
 *      允许在 ADC 启动前预调用一次，零初始电压使首次周期检查前保持保护。
 * @note 快照只复制三个电压和错误字；区外计算，发布时保留最新短路位。
 *       只发布错误字，不操作 PWM 或 PID；ADC 下一周期据此保护并清零恢复计数。
 *       无错误不直接开启或复位 PID；短路锁存不随普通故障解除。
 *       裁判时间戳沿用现有接收代码，不提供新的报文有效性判据。
 */
void PowerCtrl_UpdateErrors(void);

/**
 * @brief 25 kHz 检查底盘过流，执行状态切换及四环控制。
 * @pre ADC1 TC 先完成五路反馈处理；控制周期为 POWER_CTRL_PERIOD_US。
 *      ADC1 DMA 优先级 0、Master 为 1；计数器及预装载已运行至少一个 PWM 周期。
 * @note 短路只判断底盘电流绝对值，命中置位并转 PROTECT，同周期关断，禁止自动重启。
 *       前馈所需电压无效时直接保护并清恢复计数，不除法、不改普通错误位；
 *       普通错误仍由 Master 1 kHz 归因。原始 Vcap/Vchassis 低于 0.05 同样禁止开启。
 *       V_BAT 须在 [15, ADC_V_BAT_MAX_V]，两路电流须在各自 ADC 理论量程内；
 *       统一用有效范围拒绝异常，直接保护、清恢复计数，不新增错误位或重复检查功率乘积。
 *       PROTECT 每周期关断；有效电压时预置前馈，无效时保留旧占空比记录。
 *       错误清零且前馈连续可用的第 12501 周期复位四环并单次开启；CTRL 不重复开启。
 *       PWM、比较值写入及 PID 复位只由 ADC 上下文执行。
 *       当前电流环、电压环每周期计算并竞争；能量环与功率环保持注释。
 *       电流环启用积分限幅和条件积分，电压环保持 NONE；不增加环间输出跟踪。
 *       电流修正同时受 ±MaxOut 与总占空比边界约束，不按裁判帧更新或执行输出跟踪。
 *       开环不调用此入口；不增加采样停更、执行超期或 CAN 模式控制。
 */
void PowerCtrl_Task(void);

#endif /* SUPERPOWER_POWER_CTRL_H */
