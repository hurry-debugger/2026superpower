/**
 * @file data_scope.h
 * @brief VOFA+ JustFloat 非阻塞 DMA 波形接口。
 */
#ifndef SUPERPOWER_DATA_SCOPE_H
#define SUPERPOWER_DATA_SCOPE_H

#include "usart.h"

/** @brief 当前电流加电压档最多六路 float，每路四字节。 */
#define DATA_SCOPE_MAX_CHANNELS 6U
/** @brief 用户指定的唯一发送缓冲区容量，单位字节；不按容量发送。 */
#define DATA_SCOPE_TX_BUFFER_SIZE 128U
/** @brief JustFloat 每通道字节数，协议要求 IEEE 754 binary32。 */
#define DATA_SCOPE_CHANNEL_BYTES 4U
/** @brief JustFloat 帧尾 00 00 80 7F 的字节数。 */
#define DATA_SCOPE_TAIL_BYTES 4U

/**
 * @brief 顺序追加一通道浮点值，达到容量时忽略额外通道。
 * @param[in] data 通道数据，物理单位由 debug.h 中的档位约定。
 * @pre 仅由 DataWavePkg 调用；使用 STM32 小端四字节浮点表示。
 */
void DataScope_Get_Channel_Data(float data);

/**
 * @brief 取得所选 PID 的一致快照，追加帧尾并启动一次 DMA 发送。
 * @param[in,out] huart 已初始化且具有普通模式 TX DMA 的调试串口句柄。
 * @pre 仅由 Master 1 kHz 回调串行调用，传入 &huart1；本模块独占其发送。
 * @note DMA 忙时跳过本次调用，不改写缓冲区、不等待或排队。
 *       只在字段复制期间屏蔽中断，恢复原 PRIMASK 后再启动 DMA。
 *       启动失败仅累计私有 tx_error_count，下一次调用重新尝试，不改变功率输出。
 */
void DataWave(UART_HandleTypeDef *huart);

#endif /* SUPERPOWER_DATA_SCOPE_H */
