/**
 * @file data_scope.c
 * @brief 适配旧工程 data_scope 模块，按实际帧长发送 JustFloat 波形。
 */
#include "data_scope.h"

#include "debug.h"

#include <float.h>
#include <string.h>

#if DATA_SCOPE_MAX_CHANNELS * DATA_SCOPE_CHANNEL_BYTES + DATA_SCOPE_TAIL_BYTES > DATA_SCOPE_TX_BUFFER_SIZE
#error "DataScope frame exceeds TX buffer capacity"
#endif

typedef char DataScopeFloatSizeCheck[(sizeof(float) == DATA_SCOPE_CHANNEL_BYTES
    && FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128) ? 1 : -1];

/** @brief 工作区仅由 Master 写入；发送期间 DMA 只读 buffer。 */
static struct
{
    uint8_t buffer[DATA_SCOPE_TX_BUFFER_SIZE]; /**< 单缓冲区，DMA 忙时保持原样。 */
    uint8_t channel_count; /**< 本帧实际通道数，兼作写入位置，范围 0～6。 */
    volatile uint32_t tx_error_count; /**< 调试器观察的启动失败次数，自然回绕。 */
} s_data_scope;

void DataScope_Get_Channel_Data(float data)
{
    if(s_data_scope.channel_count < DATA_SCOPE_MAX_CHANNELS)
    {
        memcpy(&s_data_scope.buffer[s_data_scope.channel_count * DATA_SCOPE_CHANNEL_BYTES],
               &data, sizeof(data));
        s_data_scope.channel_count++;
    }
}

void DataWave(UART_HandleTypeDef *huart)
{
    /* gState 只描述发送，不能用包含裁判接收状态的合并状态判断发送空闲。 */
    if(huart->gState == HAL_UART_STATE_READY)
    {
        s_data_scope.channel_count = 0U;

        DataWavePkg();

        if(s_data_scope.channel_count > 0U)
        {
            s_data_scope.buffer[s_data_scope.channel_count * DATA_SCOPE_CHANNEL_BYTES] = 0x00U;
            s_data_scope.buffer[s_data_scope.channel_count * DATA_SCOPE_CHANNEL_BYTES + 1U] = 0x00U;
            s_data_scope.buffer[s_data_scope.channel_count * DATA_SCOPE_CHANNEL_BYTES + 2U] = 0x80U;
            s_data_scope.buffer[s_data_scope.channel_count * DATA_SCOPE_CHANNEL_BYTES + 3U] = 0x7FU;

            if(HAL_UART_Transmit_DMA(huart, s_data_scope.buffer,
                (uint16_t)(s_data_scope.channel_count * DATA_SCOPE_CHANNEL_BYTES
                    + DATA_SCOPE_TAIL_BYTES)) != HAL_OK)
                s_data_scope.tx_error_count++;
        }
    }
}
