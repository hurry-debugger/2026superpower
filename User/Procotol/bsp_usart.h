#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "usart.h"

#define   JUDGE_HUART     huart2

//串口DMA接收缓冲区长度
#define  JUDGE_DATA_LEN		300 

void USER_UART_Init(void);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
void USER_UART_IDLECallback(UART_HandleTypeDef *huart, uint16_t rx_len);

#endif

