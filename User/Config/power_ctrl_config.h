/** @file power_ctrl_config.h
 * @brief 控制参数、保护周期计数及毫秒超时；参数未就绪时禁止输出。
 * @note 编译模式在 main.h 的 USER CODE Private defines 中用注释选择。
 */
#ifndef SUPERPOWER_POWER_CTRL_CONFIG_H
#define SUPERPOWER_POWER_CTRL_CONFIG_H

/** @brief PID 参数和时序填写完成后手动置 1；不代表上板验证。 */
#define POWER_CTRL_PARAMETERS_READY (0U)
/** @brief 本轮已静态核对 CubeMX 预装载和调度配置，不代表波形或硬件已验证。 */
#define POWER_CTRL_PWM_READY (1U)
/** @brief Master 固定周期 40 us，即 25 kHz；来源为本轮用户调度约定。 */
#define POWER_CTRL_PERIOD_US (40U)
/** @brief 250 kHz 每通道十次采样对应整组周期 40 us；发布可延后一控制周期。 */
#define POWER_CTRL_SAMPLE_PERIOD_US (40U)
/** @brief 当前 170 MHz * 32 / 21760 对应 PWM 周期 4 us，来源为 CubeMX 配置。 */
#define POWER_CTRL_PWM_PERIOD_US (4U)

/** @brief 用户确认的电容平均电流限值 A，理想功率换算模型。 */
#define POWER_CTRL_CAP_CURRENT_MAX_A (15.0f)
/** @brief 用户确认的电容硬过压阈值 V。 */
#define POWER_CTRL_CAP_OVERVOLTAGE_V (30.0f)
/** @brief 全局放电功率下限 W；另受实时电容功率限制。 */
#define POWER_CTRL_POWER_MIN_W (-450.0f)
/** @brief 电容充电目标 V，用户确认留出 1 V 余量。 */
#define POWER_CTRL_CAP_TARGET_V (29.0f)
/** @brief 港科 90% 入口比例对应本轮 26.1 V。 */
#define POWER_CTRL_VOLTAGE_COMPETE_V (26.1f)
/** @brief 旧工程参考底盘过压 V，待实板验证。 */
#define POWER_CTRL_CHASSIS_OVERVOLTAGE_V (43.0f)
/** @brief 底盘过压恢复 V，必须严格低于此值。 */
#define POWER_CTRL_CHASSIS_RECOVER_V (33.0f)
/** @brief 旧工程参考电池欠压 V。 */
#define POWER_CTRL_BAT_UNDERVOLTAGE_V (10.0f)
/** @brief 连续控制周期数；每次 Task 更新一次，不是毫秒。 */
#define POWER_CTRL_CAP_OVERVOLTAGE_CYCLES (10U)
#define POWER_CTRL_CHASSIS_OVERVOLTAGE_CYCLES (1U)
#define POWER_CTRL_BAT_UNDERVOLTAGE_CYCLES (10U)
/** @brief 港科 TIM2 名义 1 ms 的 500 次冷却，换算为 500 ms。 */
#define POWER_CTRL_RESTART_DELAY_MS (500U)
/** @brief 旧工程参考裁判超时 ms，仅上场检查。 */
#define POWER_CTRL_REFEREE_TIMEOUT_MS (300U)
/** @brief 缓冲能量目标 J，旧工程参考初值。 */
#define POWER_CTRL_ENERGY_TARGET_J (45.0f)
/** @brief 无裁判初始目标 W，可通过接口设为负值。 */
#define POWER_CTRL_TEST_POWER_W (80.0f)

#endif /* SUPERPOWER_POWER_CTRL_CONFIG_H */
