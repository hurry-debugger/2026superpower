/**
 * @file power_ctrl_config.h
 * @brief 25 kHz 控制参数、1 kHz 保护累计及恢复周期。
 * @note 阈值来自用户确认，实际保护能力待上板验证；编译模式由 main.h 选择。
 */
#ifndef SUPERPOWER_POWER_CTRL_CONFIG_H
#define SUPERPOWER_POWER_CTRL_CONFIG_H

/** @brief ADC1 每十次扫描产生 TC，控制周期 40 us，即 25 kHz。 */
#define POWER_CTRL_PERIOD_US (40U)
/** @brief CubeMX 配置：170 MHz * 32 / 21760，PWM 周期 4 us。 */
#define POWER_CTRL_PWM_PERIOD_US (4U)

/** @brief 用户指定的正常电容平均目标参考上限 A；按电压比换算，不含效率补偿。 */
#define POWER_CTRL_CAP_CURRENT_MAX_A (13.0f)
/** @brief 用户指定的 B 端峰值设计约束 A；仅记录约束，本轮未实现峰值保护。 */
#define POWER_CTRL_CAP_CURRENT_PEAK_A (15.0f)
/** @brief 港科 RM2024 参考分界 V；低于此值仅允许充电目标，不主动请求放电。 */
#define POWER_CTRL_CAP_LOW_VOLTAGE_V (5.0f)
/** @brief 港科 RM2024 参考分界 V；达到后恢复正常平均目标，不是硬欠压门槛。 */
#define POWER_CTRL_CAP_NORMAL_VOLTAGE_V (12.0f)
/** @brief 用户确认的低压充电平均目标上限 A，参考港科 RM2024；本板能力待测。 */
#define POWER_CTRL_CAP_LOW_CHARGE_CURRENT_A (4.0f)
/** @brief 全局放电功率下限 W，另受实时电容功率限制。 */
#define POWER_CTRL_POWER_MAX_W (450.0f)
/** @brief 用户确认的电容充电目标 V，独立于硬过压阈值。 */
#define POWER_CTRL_CAP_TARGET_V (29.0f)
/** @brief 港科 90% 入口比例对应 26.1 V，电压环开始竞争。 */
#define POWER_CTRL_VOLTAGE_COMPETE_V (26.1f)

/** @brief 用户确认的底盘、电容共用硬过压阈值 V，严格超过即保护。 */
#define POWER_CTRL_OVERVOLTAGE_V (31.0f)
/** @brief 底盘、电池共用输入下限 V；电池不设置物理电压上限。 */
#define POWER_CTRL_INPUT_MIN_V (15.0f)
/** @brief 用户指定的底盘过流关断阈值 A：450 W / 15 V，不附加低压条件。 */
#define POWER_CTRL_CHASSIS_CURRENT_MAX_A \
    (POWER_CTRL_POWER_MAX_W / POWER_CTRL_INPUT_MIN_V)

/** @brief 港科 RM2024 底盘过压分档 V，实现按高到低判断，均为严格大于。 */
#define POWER_CTRL_OVERVOLTAGE_LEVEL_1_V (27.0f)
#define POWER_CTRL_OVERVOLTAGE_LEVEL_2_V (28.0f)
#define POWER_CTRL_OVERVOLTAGE_LEVEL_3_V (29.0f)
#define POWER_CTRL_OVERVOLTAGE_LEVEL_4_V (30.0f)
/** @brief 各档每次 1 kHz 检查的加权步长，无量纲，不是 ADC 周期数。 */
#define POWER_CTRL_OVERVOLTAGE_STEP_1 (1U)
#define POWER_CTRL_OVERVOLTAGE_STEP_2 (5U)
#define POWER_CTRL_OVERVOLTAGE_STEP_3 (25U)
#define POWER_CTRL_OVERVOLTAGE_STEP_4 (125U)
/** @brief 港科 RM2024 加权累计门槛；达到后饱和，底盘不高于 27 V 时清零。 */
#define POWER_CTRL_OVERVOLTAGE_ACCUM_LIMIT (300U)

/** @brief 用户指定的闭环连续无错误恢复时间 ms。 */
#define POWER_CTRL_RESTART_DELAY_MS (500U)
/** @brief 换算为 ADC 控制周期；当前门槛 12500 次，严格超过才恢复。 */
#define POWER_CTRL_RESTART_DELAY_CYCLES \
    (POWER_CTRL_RESTART_DELAY_MS * 1000U / POWER_CTRL_PERIOD_US)
/** @brief 裁判共用接收时间戳超时 ms，仅闭环检查。 */
#define POWER_CTRL_REFEREE_TIMEOUT_MS (300U)
/** @brief 缓冲能量目标 J，旧工程参考初值。 */
#define POWER_CTRL_ENERGY_TARGET_J (45.0f)

#endif /* SUPERPOWER_POWER_CTRL_CONFIG_H */
