
/**
 * @file bsp_dwt.h
 * @brief STM32G4 DWT 执行时间测量模块。
 */

#ifndef SUPERPOWER_BSP_DWT_H
#define SUPERPOWER_BSP_DWT_H

#include <stdint.h>

/* ==================== 公共定义 ==================== */

#define DWT_SLOT_COUNT 16U

/**< 各计时槽执行耗时，单位 us，可通过 Keil Watch 直接查看。 */
extern volatile float g_dwt_us[DWT_SLOT_COUNT];

/* ==================== 公共接口 ==================== */

/**
 * @brief 初始化 DWT 周期计数器。
 * @note 系统时钟配置完成后调用一次。
 */
void Bsp_DWT_Init(void);

/**
 * @brief 开始计时。
 * @param[in] slot 计时槽编号，范围 0~15。
 * @pre slot < DWT_SLOT_COUNT。
 * @note 同一计时槽不允许嵌套或并发使用。
 */
void Bsp_DWT_Start(uint32_t slot);

/**
 * @brief 停止计时并更新耗时数组。
 * @param[in] slot 计时槽编号，范围 0~15。
 * @pre 已调用对应计时槽的 Bsp_DWT_Start()。
 * @note 耗时写入 g_dwt_us[slot]，单位 us。
 */
void Bsp_DWT_Stop(uint32_t slot);

#endif /* SUPERPOWER_BSP_DWT_H */
