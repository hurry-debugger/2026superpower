/**
 * @file bsp_usart.h
 * @brief 裁判串口 DMA 接收、Master 解析及主循环恢复。
 * @note 由 Taurus 接收流程适配，不包含遥控、图传、屏幕、发送或打印。
 */
#ifndef SUPERPOWER_BSP_USART_H
#define SUPERPOWER_BSP_USART_H

#include "main.h"

#if defined(HAL_UART_MODULE_ENABLED)

/**
 * @brief 绑定唯一裁判 UART 并启动一次 DMA 接收和空闲中断。
 * @param[in,out] huart 已完成 CubeMX 初始化、具有有效 RX DMA 的 UART 句柄。
 * @pre RX DMA 必须为普通模式；仅在启动阶段调用一次，不与接收或 Process 并发。
 * @note 不初始化外设、不指定串口编号。HAL 失败时调用 Error_Handler()。
 *       绑定 USART1 / DMA1_Channel3，IRQ 和 Master 已接入调度。
 */
void USER_UART_Init(UART_HandleTypeDef *huart);

/**
 * @brief 在绑定 UART 的中断中处理空闲事件，停止 DMA 并移交稳定缓冲区。
 * @param[in,out] huart 触发 UART 中断的句柄。
 * @note 在对应 UART IRQ 中先调用本函数，再调用 HAL_UART_IRQHandler()；
 *       此函数不解析协议，也不写控制层读取的协议对象。
 */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);

/**
 * @brief 记录停止接收后的长度，交给前台处理。
 * @param[in,out] huart 已绑定且 RX DMA 已停止的句柄。
 * @param[in] rx_len 本次有效字节数，范围 0 至 JUDGE_DATA_LEN。
 * @note 仅供本模块的空闲中断路径调用；不允许另一个生产者并发调用。
 */
void USER_UART_IDLECallback(UART_HandleTypeDef *huart, uint16_t rx_len);

/**
 * @brief 解析待处理缓冲区后重启接收，无待处理数据时直接返回。
 * @param[in] now_ms 与 HAL_GetTick() 同源的当前毫秒时刻。
 * @pre 已完成 USER_UART_Init()，与 PowerCtrl_Task() 在同一串行上下文调用。
 * @note DMA 停止后才解析，期间不重写接收缓冲区；解析后重启 DMA。
 *       USART/RX DMA IRQ 仍 active 或前台恢复时跳过；失败请求主循环恢复，不阻塞 Master。
 *       调用时间属于接收处理时间，不是硬件逐帧到达时间；前台调度延迟需验证。
 */
void USER_UART_Process(uint32_t now_ms);

/**
 * @brief 主循环处理 UART 错误和重启失败，不解析协议、不刷新裁判有效时间。
 * @pre 主循环唯一调用；Master 通过所有权标志避开恢复中的 HAL 句柄。
 * @note 暂时关闭 USART1/RX DMA IRQ 并恢复原使能状态，允许前台 HAL 终止等待。
 *       未成功则保留请求，下次主循环重试；满 150 字节可移交 Master。
 */
void USER_UART_Recover(void);

#endif /* HAL_UART_MODULE_ENABLED */

#endif /* SUPERPOWER_BSP_USART_H */
