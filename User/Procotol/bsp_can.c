/**
 * @file bsp_can.c
 * @brief Taurus 双 ID 过滤、回调分发和非阻塞发送的 FDCAN1 适配。
 * @note 仅支持标准经典数据帧；不导入其他 CAN 实例、RTOS 或电机业务。
 */
#include "bsp_can.h"

#include <stddef.h>

/** @brief Taurus 过滤列表终止值，超出标准 ID 范围。 */
#define BSP_CAN_ID_END (0xFFFU)
/** @brief 本工程 G4 HAL 固定的每个 RX FIFO 深度，来源 SRAMCAN_RF0/1_NBR。 */
#define BSP_CAN_RX_FIFO_DEPTH (3U)

static void (*s_rx_callback)(uint32_t, uint8_t *);

static void BspCan_ReadFifo(FDCAN_HandleTypeDef *hfdcanx, uint32_t fifo);

void Fdcanx_Init(FDCAN_HandleTypeDef *hfdcanx, const uint32_t fdcanx_id[],
                void (*callback)(uint32_t, uint8_t *))
{
    uint32_t i = 0U;
    uint32_t filter_index = 0U;

    if(hfdcanx != &hfdcan1)
    {
        Error_Handler();
        return;
    }
    /* 在使能通知和启动前完成注册，避免首帧到达时尚无消费者。 */
    s_rx_callback = callback;
    while(fdcanx_id[i] != BSP_CAN_ID_END)
    {
        FDCAN_FilterTypeDef filter = {0};
        if(filter_index >= hfdcanx->Init.StdFiltersNbr || fdcanx_id[i] > 0x7FFU)
        {
            Error_Handler();
            return;
        }
        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterIndex = filter_index;
        filter.FilterType = FDCAN_FILTER_DUAL;
        filter.FilterID1 = fdcanx_id[i++];
        filter.FilterID2 = filter.FilterID1;
        if(fdcanx_id[i] != BSP_CAN_ID_END)
        {
            if(fdcanx_id[i] > 0x7FFU)
            {
                Error_Handler();
                return;
            }
            filter.FilterID2 = fdcanx_id[i++];
        }
        filter.FilterConfig = (filter_index % 2U == 0U)
            ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
        if(HAL_FDCAN_ConfigFilter(hfdcanx, &filter) != HAL_OK)
        {
            Error_Handler();
            return;
        }
        filter_index++;
    }
    if(HAL_FDCAN_ConfigGlobalFilter(hfdcanx, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK
       || HAL_FDCAN_ActivateNotification(hfdcanx, FDCAN_IT_RX_FIFO0_NEW_MESSAGE
                                        | FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0U) != HAL_OK
       || HAL_FDCAN_Start(hfdcanx) != HAL_OK)
    {
        Error_Handler();
    }
}

HAL_StatusTypeDef Fdcanx_Transmit(FDCAN_HandleTypeDef *hfdcanx, uint32_t tx_id,
                                uint8_t *tx_data)
{
    FDCAN_TxHeaderTypeDef header = {0};
    if(hfdcanx != &hfdcan1 || tx_data == NULL || tx_id > 0x7FFU)
        return HAL_ERROR;
    header.Identifier = tx_id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcanx) == 0U)
        return HAL_BUSY;
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcanx, &header, tx_data);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcanx, uint32_t interrupts)
{
    if(hfdcanx == &hfdcan1 && (interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
        BspCan_ReadFifo(hfdcanx, FDCAN_RX_FIFO0);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcanx, uint32_t interrupts)
{
    if(hfdcanx == &hfdcan1 && (interrupts & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U)
        BspCan_ReadFifo(hfdcanx, FDCAN_RX_FIFO1);
}

static void BspCan_ReadFifo(FDCAN_HandleTypeDef *hfdcanx, uint32_t fifo)
{
    /* 最多处理硬件深度帧数，持续总线流量不能使中断无限循环。 */
    for(uint32_t i = 0U; i < BSP_CAN_RX_FIFO_DEPTH; i++)
    {
        FDCAN_RxHeaderTypeDef header = {0};
        uint8_t data[64] = {0};
        if(HAL_FDCAN_GetRxFifoFillLevel(hfdcanx, fifo) == 0U)
            break;
        if(HAL_FDCAN_GetRxMessage(hfdcanx, fifo, &header, data) != HAL_OK)
            break;
        if(header.IdType == FDCAN_STANDARD_ID && header.RxFrameType == FDCAN_DATA_FRAME
           && header.FDFormat == FDCAN_CLASSIC_CAN
           && header.DataLength >= FDCAN_DLC_BYTES_1 && header.DataLength <= FDCAN_DLC_BYTES_8
           && s_rx_callback != NULL)
            s_rx_callback(header.Identifier, data);
    }
}
