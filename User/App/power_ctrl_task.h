/**
 * @file power_ctrl_task.h
 * @brief ADC1 DMA 25 kHz 过流/控制与 Master 1 kHz 普通保护/CAN/DMA 波形调度。
 */
#ifndef SUPERPOWER_POWER_CTRL_TASK_H
#define SUPERPOWER_POWER_CTRL_TASK_H

/**
 * @brief 初始化控制、采样和通信，并一次启动 Master/A/B 计数器及选定中断。
 * @pre CubeMX 外设和 UID 匹配、ADC 硬件自校准完成；计数器尚未启动，输出关闭。
 * @pre CubeMX 已配置 ADC1 DMA 优先级 0、Master 优先级 1 及 1 kHz MREP，A/B NVIC 关闭。
 *      时基、预装载和采样触发由 CubeMX 配置，不在此重复核对。
 * @note 不改写外设配置；HAL 启动失败进入 Error_Handler()。
 *       共用初始化包含四环初始化；闭环启动设置校准汇总并预先检查普通错误。
 *       ADC 闭环回调执行采样、过流及控制；Master 依次更新普通错误、发送 CAN、启动 USART1 波形 DMA。
 *       波形只在短临界区复制 PID 字段，DMA 忙时跳帧，不等待发送完成。
 *       Master 不开启 PWM 或复位 PID；跨优先级快照及发布使用短临界区。
 *       开环每次 ADC1 回调都调用固定 PWM 开启，不处理采样、不响应远程保护。
 *       Master 在开环仍更新错误字，但无闭环状态机据此关断；须外部隔离驱动。
 */
void SuperPower_Init(void);

#endif /* SUPERPOWER_POWER_CTRL_TASK_H */
