/**
 * @file prot_supercap.h
 * @brief 兼容旧超电工程的经典 CAN 报文与底盘模式。
 */
#ifndef SUPERPOWER_PROT_SUPERCAP_H
#define SUPERPOWER_PROT_SUPERCAP_H

#include <stdint.h>

#define SUPERCAP_CAN_RX_MODE_ID (0x010U)
#define SUPERCAP_CAN_TX_MEASUREMENT_ID (0x100U)
#define SUPERCAP_CAN_TX_STATE_ID (0x101U)
#define SUPERCAP_CAN_REMOTE_PROTECT (1U)

/** @brief 旧协议故障字节的已实现位；其他旧位不虚构故障检测。 */
#define SUPERCAP_CAN_FAULT_CAP_OVERVOLTAGE (1U << 0)
#define SUPERCAP_CAN_FAULT_CHASSIS_OVERVOLTAGE (1U << 2)
#define SUPERCAP_CAN_FAULT_BAT_UNDERVOLTAGE (1U << 3)
#define SUPERCAP_CAN_FAULT_REFEREE (1U << 7)
#define SUPERCAP_CAN_STATE_NORMAL (0U)
#define SUPERCAP_CAN_STATE_PROTECT (1U)
#define SUPERCAP_CAN_STATE_RESTART (2U)

/** @brief 协议接收值与发送统计；仅协议实现写入，其他模块只读。 */
typedef struct
{
    volatile uint8_t chassis_mode; /**< RX IRQ 发布的原子字节；1 请求保护，其他值解除。 */
    uint32_t tx_busy_count; /**< Timer B 发送时 FIFO 满而丢弃的帧数，自然回绕。 */
    uint32_t tx_error_count; /**< Timer B 发送 HAL 失败的帧数，自然回绕。 */
} SupercapCan_t;

extern SupercapCan_t g_supercap_can;

/** @brief 启动前清零协议状态，使用 Taurus BSP 绑定并启动 FDCAN1。 */
void SupercapCan_Init(void);
/**
 * @brief 每毫秒分别尝试发送测量和状态帧，不等待、不重试。
 * @param[in] cap_voltage_v 电容电压 V，按 IEEE 754 binary32 小端传输。
 * @param[in] chassis_current_a 底盘电流 A；旧源码实际传入的不是电容电流。
 * @param[in] faults 已映射为旧定义的故障字节。
 * @param[in] state 已映射为旧定义的 0/1/2 状态字节。
 * @pre Timer B 唯一调用；四个输入已由应用层取得一致快照。
 * @note 状态帧剩余六字节固定为零；HAL_OK 只表示入队成功。
 */
void SupercapCan_SendFeedback(float cap_voltage_v, float chassis_current_a,
                             uint8_t faults, uint8_t state);

#endif /* SUPERPOWER_PROT_SUPERCAP_H */
