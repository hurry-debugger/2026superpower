/**
 * @file prot_supercap.c
 * @brief 旧 SuperCap2026_7_17 线上格式与 Taurus BSP 的对接。
 * @note 读取采样与 App 控制状态并取得一致快照；不导入旧控制参数或 FD 报文。
 */
#include "prot_supercap.h"

#include "bsp_can.h"
#include "bsp_adc.h"
#include "power_ctrl.h"

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

void SupercapCan_SendStatus(void)
{
    float cap_voltage_v;
    float chassis_current_a;
    PowerCtrlError_t errors;
    PowerCtrlState_e state;
    uint8_t faults = 0U;
    uint8_t wire_state = SUPERCAP_CAN_STATE_PROTECT;

    /* 仅复制最小反馈，恢复调用前 PRIMASK；编码与 HAL 发送均在临界区之外。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    cap_voltage_v = g_bsp_adc_feedback.adc_convert[V_CAP];
    chassis_current_a = g_bsp_adc_feedback.adc_convert[I_CHASSIS];
    errors = g_power_ctrl.errors;
    state = g_power_ctrl.state;
    __DMB();
    __set_PRIMASK(primask);

    if(errors.bits.cap_overvoltage)
        faults |= SUPERCAP_CAN_FAULT_CAP_OVERVOLTAGE;
    if(errors.bits.chassis_overvoltage)
        faults |= SUPERCAP_CAN_FAULT_CHASSIS_OVERVOLTAGE;
    if(errors.bits.bat_undervoltage)
        faults |= SUPERCAP_CAN_FAULT_BAT_UNDERVOLTAGE;
    if(errors.bits.referee)
        faults |= SUPERCAP_CAN_FAULT_REFEREE;
    if(state == CTRL_MODE)
        wire_state = SUPERCAP_CAN_STATE_NORMAL;
    SupercapCan_SendFeedback(cap_voltage_v, chassis_current_a, faults, wire_state);
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
