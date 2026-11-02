/**
 * @file bsp_can.h
 * @brief 由 Taurus BSP 适配的单路 FDCAN1 经典 CAN 收发接口。
 */
#ifndef SUPERPOWER_BSP_CAN_H
#define SUPERPOWER_BSP_CAN_H

#include "fdcan.h"

/**
 * @brief 配置双 ID 精确过滤并启动唯一 FDCAN1。
 * @param[in,out] hfdcanx 已由 CubeMX 初始化的 FDCAN1，标准过滤器数量须足够。
 * @param[in] fdcanx_id 以 0xFFF 结束的标准 ID 数组；最后一个单 ID 自动成对。
 * @param[in] callback 中断接收回调；数据为补零至 8 字节的临时数组，不得保存指针。
 * @pre 仅在启动阶段、功率输出关闭时调用一次；ID 范围 0～0x7FF。
 * @note 沿用 Taurus 函数名；初始化 HAL 失败进入 Error_Handler。
 */
void Fdcanx_Init(FDCAN_HandleTypeDef *hfdcanx, const uint32_t fdcanx_id[],
                void (*callback)(uint32_t, uint8_t *));

/**
 * @brief 尝试发送一帧标准经典 CAN，固定 8 字节，不等待、不重试。
 * @param[in,out] hfdcanx 已启动的 FDCAN1。
 * @param[in] tx_id 标准 ID，范围 0～0x7FF。
 * @param[in] tx_data 至少 8 字节，HAL 调用结束前稳定。
 * @return 沿用 Taurus 的 HAL 状态：成功入队为 HAL_OK，队列满为 HAL_BUSY。
 * @note 仅 Timer B 调用；返回成功不代表总线已经发送或得到确认。
 */
HAL_StatusTypeDef Fdcanx_Transmit(FDCAN_HandleTypeDef *hfdcanx, uint32_t tx_id,
                                uint8_t *tx_data);

#endif /* SUPERPOWER_BSP_CAN_H */
