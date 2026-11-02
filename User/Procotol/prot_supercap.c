/**
 * @file prot_supercap.c
 * @brief 旧 SuperCap2026_7_17 线上格式与 Taurus BSP 的对接。
 * @note 不依赖 App；不导入旧控制参数、FD 报文或未初始化尾字节。
 */
#include "prot_supercap.h"

#include "bsp_can.h"

#include <float.h>
#include <string.h>

typedef char SupercapCanFloatSizeCheck[(sizeof(float) == 4U && FLT_RADIX == 2
    && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128) ? 1 : -1];

SupercapCan_t g_supercap_can;

static void SupercapCan_Receive(uint32_t rx_id, uint8_t *data);
static void SupercapCan_EncodeFloat(uint8_t *data, float value);
static void SupercapCan_Send(uint32_t id, uint8_t *data);

void SupercapCan_Init(void)
{
    static const uint32_t ids[] = {SUPERCAP_CAN_RX_MODE_ID, 0xFFFU};
    memset(&g_supercap_can, 0, sizeof(g_supercap_can));
    Fdcanx_Init(&hfdcan1, ids, SupercapCan_Receive);
}

void SupercapCan_SendFeedback(float cap_voltage_v, float chassis_current_a,
                             uint8_t faults, uint8_t state)
{
    uint8_t measurement[8];
    uint8_t status[8] = {0};
    SupercapCan_EncodeFloat(measurement, cap_voltage_v);
    SupercapCan_EncodeFloat(&measurement[4], chassis_current_a);
    status[0] = faults;
    status[1] = state;
    SupercapCan_Send(SUPERCAP_CAN_TX_MEASUREMENT_ID, measurement);
    SupercapCan_Send(SUPERCAP_CAN_TX_STATE_ID, status);
}

static void SupercapCan_Receive(uint32_t rx_id, uint8_t *data)
{
    if(rx_id == SUPERCAP_CAN_RX_MODE_ID)
        g_supercap_can.chassis_mode = data[0];
}

static void SupercapCan_EncodeFloat(uint8_t *data, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    for(uint32_t i = 0U; i < 4U; i++)
        data[i] = (uint8_t)(bits >> (8U * i));
}

static void SupercapCan_Send(uint32_t id, uint8_t *data)
{
    HAL_StatusTypeDef status = Fdcanx_Transmit(&hfdcan1, id, data);
    if(status == HAL_BUSY)
        g_supercap_can.tx_busy_count++;
    else if(status != HAL_OK)
        g_supercap_can.tx_error_count++;
}
