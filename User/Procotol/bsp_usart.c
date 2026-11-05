#include "bsp_usart.h"
#include "stdlib.h"
#include "string.h"
#include "prot_judge.h"

uint8_t	judge_dma_buf[JUDGE_DATA_LEN];


 /**
  * @brief  各个串口功能函数
  * @param 	UART_HandleTypeDef *huart
  * @retval 无
  */
void USER_UART_IDLECallback(UART_HandleTypeDef *huart, uint16_t rx_len)
{
	/* 裁判系统串口 */
    if(huart == &JUDGE_HUART)
    {
        judge_get_data(judge_dma_buf, rx_len);
        HAL_UART_Receive_DMA(huart, judge_dma_buf, sizeof(judge_dma_buf));
    }
}


/**
  * @brief 串口空闲中断（需在it.c中每个串口的中断中调用该函数）
  * @param UART_HandleTypeDef *huart
  * @retval 无
  */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    uint16_t rx_len = 0U;

	if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   //判断是否是空闲中断
	{
        if (huart->hdmarx != NULL)
        {
            rx_len = (uint16_t)(huart->RxXferSize - __HAL_DMA_GET_COUNTER(huart->hdmarx));
        }

		__HAL_UART_CLEAR_IDLEFLAG(huart);                     //清除空闲中断标志（否则会一直不断进入中断）
		HAL_UART_AbortReceive(huart);							//停止本次DMA接收
        USER_UART_IDLECallback(huart, rx_len);                        //调用串口功能函数
	}
}


/**
* @brief 串口初始化:使能串口空闲中断,开启串口DMA接收
* @param  无
* @retval 无
*/
void USER_UART_Init(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&JUDGE_HUART);
    __HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&JUDGE_HUART, judge_dma_buf, sizeof(judge_dma_buf));
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE)
    {
        __HAL_UART_FLUSH_DRREGISTER(huart);  //读DR寄存器，就可以清除ORE错误标志位
    }

		HAL_UART_DMAStop(huart);		
		HAL_UART_Receive_DMA(&JUDGE_HUART, judge_dma_buf, sizeof(judge_dma_buf));
}

