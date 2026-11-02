/**
 * @file power_ctrl_task.h
 * @brief HRTIM Master 25 kHz 控制与 Timer B 1 kHz CAN 调度。
 */
#ifndef SUPERPOWER_POWER_CTRL_TASK_H
#define SUPERPOWER_POWER_CTRL_TASK_H

#include <stdbool.h>
#include <stdint.h>

/** @brief 调度诊断；启动主线程初始化，Master 更新超期次数，其他上下文只读。 */
typedef struct
{
    bool configuration_ready; /**< 外设时序、请求和优先级满足接入条件。 */
    uint32_t master_overrun_count; /**< 检测到下一个 MREP 已到达的次数，自然回绕。 */
} PowerCtrlTask_t;

extern PowerCtrlTask_t g_power_ctrl_task;

/**
 * @brief 初始化控制、采样和通信，并一次启动 Master/A/B 计数器及选定中断。
 * @pre CubeMX 外设和 UID 匹配、ADC 硬件自校准完成；计数器尚未启动，输出关闭。
 * @note 配置不满足约定时保持停机，可由 configuration_ready 观察；不改写外设配置。
 *       PID/校准门槛仍由 PowerCtrl 检查，不因调度初始化而放行输出。
 */
void PowerCtrlTask_Init(void);

#endif /* SUPERPOWER_POWER_CTRL_TASK_H */
