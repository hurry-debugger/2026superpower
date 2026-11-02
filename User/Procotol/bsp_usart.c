/**
 * @file bsp_usart.c
 * @brief 稳定接收缓冲由 IRQ 移交 Master，硬件错误由前台恢复。
 */
#include "bsp_usart.h"

#if defined(HAL_UART_MODULE_ENABLED)
#include "prot_judge.h"
#include "usart.h"

#include <stdbool.h>

static UART_HandleTypeDef *s_judge_huart;
static uint8_t s_judge_dma_buf[JUDGE_DATA_LEN];
static volatile uint16_t s_pending_rx_len;
static volatile bool s_rx_pending;
static volatile bool s_idle_abort_pending;
static volatile bool s_recover_requested;
static volatile bool s_foreground_active;

static bool UserUart_IrqActive(void);
static void UserUart_RequestRecovery(void);

void USER_UART_Init(UART_HandleTypeDef *huart)
{
    if(huart != &huart1 || huart->hdmarx == NULL
       || huart->hdmarx->Instance != DMA1_Channel3 || huart->hdmarx->Init.Mode != DMA_NORMAL)
    {
        Error_Handler();
        return;
    }
    s_judge_huart = huart;
    s_pending_rx_len = 0U;
    s_rx_pending = false;
    s_idle_abort_pending = false;
    s_recover_requested = false;
    s_foreground_active = false;
    __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    if(HAL_UART_Receive_DMA(huart, s_judge_dma_buf, sizeof(s_judge_dma_buf)) != HAL_OK)
    {
        Error_Handler();
        return;
    }
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(huart != s_judge_huart || s_judge_huart == NULL)
        return;
    if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == RESET
       || __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) == RESET)
        return;
    __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    if(s_recover_requested || s_foreground_active || s_rx_pending)
        return;
    s_pending_rx_len = (uint16_t)(huart->RxXferSize - __HAL_DMA_GET_COUNTER(huart->hdmarx));
    s_idle_abort_pending = true;
    /* 异步终止，DMA 停止及 HAL 状态恢复后才发布 pending。 */
    if(HAL_UART_AbortReceive_IT(huart) != HAL_OK)
        UserUart_RequestRecovery();
}

void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == s_judge_huart && s_idle_abort_pending)
    {
        s_idle_abort_pending = false;
        USER_UART_IDLECallback(huart, s_pending_rx_len);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == s_judge_huart)
        USER_UART_IDLECallback(huart, sizeof(s_judge_dma_buf));
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == s_judge_huart)
        UserUart_RequestRecovery();
}

void USER_UART_IDLECallback(UART_HandleTypeDef *huart, uint16_t rx_len)
{
    if(huart != s_judge_huart || s_judge_huart == NULL)
        return;
    __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
    if(rx_len > sizeof(s_judge_dma_buf))
    {
        UserUart_RequestRecovery();
        return;
    }
    if(s_recover_requested || s_foreground_active)
        return;
    s_pending_rx_len = rx_len;
    __DMB();
    s_rx_pending = true;
}

void USER_UART_Process(uint32_t now_ms)
{
    if(s_judge_huart == NULL || s_foreground_active || s_recover_requested
       || UserUart_IrqActive() || !s_rx_pending)
        return;
    __DMB();
    if(s_pending_rx_len > 0U)
        (void)judge_get_data(s_judge_dma_buf, s_pending_rx_len, now_ms);
    s_rx_pending = false;
    s_pending_rx_len = 0U;
    if(HAL_UART_Receive_DMA(s_judge_huart, s_judge_dma_buf, sizeof(s_judge_dma_buf)) != HAL_OK)
    {
        UserUart_RequestRecovery();
        return;
    }
    __HAL_UART_CLEAR_IDLEFLAG(s_judge_huart);
    __HAL_UART_ENABLE_IT(s_judge_huart, UART_IT_IDLE);
}

void USER_UART_Recover(void)
{
    if(s_judge_huart == NULL || !s_recover_requested)
        return;
    /* 标志先发布，Master 从此避开句柄；前台不会与已运行的 Master 并发。 */
    s_foreground_active = true;
    __DMB();
    uint32_t uart_enabled = NVIC_GetEnableIRQ(USART1_IRQn);
    uint32_t dma_enabled = NVIC_GetEnableIRQ(DMA1_Channel3_IRQn);
    NVIC_DisableIRQ(USART1_IRQn);
    NVIC_DisableIRQ(DMA1_Channel3_IRQn);
    __DSB();
    __ISB();
    __HAL_UART_DISABLE_IT(s_judge_huart, UART_IT_IDLE);
    s_rx_pending = false;
    s_idle_abort_pending = false;
    s_pending_rx_len = 0U;
    if(HAL_UART_AbortReceive(s_judge_huart) == HAL_OK)
    {
        __HAL_UART_CLEAR_FLAG(s_judge_huart, UART_CLEAR_PEF | UART_CLEAR_FEF
            | UART_CLEAR_NEF | UART_CLEAR_OREF | UART_CLEAR_IDLEF);
        NVIC_ClearPendingIRQ(USART1_IRQn);
        NVIC_ClearPendingIRQ(DMA1_Channel3_IRQn);
        if(HAL_UART_Receive_DMA(s_judge_huart, s_judge_dma_buf, sizeof(s_judge_dma_buf)) == HAL_OK)
        {
            s_recover_requested = false;
            __HAL_UART_ENABLE_IT(s_judge_huart, UART_IT_IDLE);
        }
    }
    /* 先释放所有权再恢复 IRQ，且两步间不允许 Master 或 UART 插入。
     * 否则 pending IDLE 可能在 foreground_active 仍为真时被清除并永久关闭。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_foreground_active = false;
    __DMB();
    if(uart_enabled != 0U)
        NVIC_EnableIRQ(USART1_IRQn);
    if(dma_enabled != 0U)
        NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    __set_PRIMASK(primask);
}

static bool UserUart_IrqActive(void)
{
    return NVIC_GetActive(USART1_IRQn) != 0U || NVIC_GetActive(DMA1_Channel3_IRQn) != 0U;
}

static void UserUart_RequestRecovery(void)
{
    s_idle_abort_pending = false;
    s_rx_pending = false;
    __DMB();
    s_recover_requested = true;
}

#endif /* HAL_UART_MODULE_ENABLED */
