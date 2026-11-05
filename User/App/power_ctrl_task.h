/**
 * @file power_ctrl_task.h
 * @brief ADC1 DMA 25 kHz 控制与 HRTIM Master 1 kHz CAN 调度。
 */
#ifndef SUPERPOWER_POWER_CTRL_TASK_H
#define SUPERPOWER_POWER_CTRL_TASK_H

/**
 * @brief 初始化控制、采样和通信，并一次启动 Master/A/B 计数器及选定中断。
 * @pre CubeMX 外设和 UID 匹配、ADC 硬件自校准完成；计数器尚未启动，输出关闭。
 * @pre CubeMX 已配置 ADC1 DMA 优先级 0、Master 优先级 1 及 1 kHz MREP，A/B NVIC 关闭。
 *      时基、预装载和采样触发由 CubeMX 配置，不在此重复核对。
 * @note 不改写外设配置；HAL 启动失败进入 Error_Handler()。
 *       PID/校准门槛仍由 PowerCtrl 检查，不因调度初始化而放行输出。
 */
void PowerCtrlTask_Init(void);

#endif /* SUPERPOWER_POWER_CTRL_TASK_H */
