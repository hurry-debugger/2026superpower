/** @file bsp_referee_data.h
 * @brief 已解析裁判数据的单写入者存储，不实现 USART 或协议解析。
 */
#ifndef SUPERPOWER_BSP_REFEREE_DATA_H
#define SUPERPOWER_BSP_REFEREE_DATA_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float power_limit_w; /**< 功率上限 W，有限且非负。 */
    float buffer_energy_j; /**< 缓冲能量 J，有限且非负。 */
    bool valid; /**< 启动 false；收到完整有效数据后才能为 true。 */
    uint32_t update_ms; /**< 最后一次有效报文时间，单位 ms。 */
} BspRefereeData_t;

/** @brief 所属 .c 唯一写入，其他模块按约定只读；extern 不是语言级写保护。 */
extern BspRefereeData_t g_bsp_referee_data;

/**
 * @brief 发布已通过协议校验的数据；无效数值标记失效且不刷新有效时间。
 * @param[in] power_limit_w 功率上限 W。
 * @param[in] buffer_energy_j 缓冲能量 J。
 * @param[in] now_ms 与 HAL_GetTick() 同源的毫秒时刻。
 * @note 唯一串行调用者先发布再控制；不得从异步中断并发写入。
 */
void BspRefereeData_Publish(float power_limit_w, float buffer_energy_j, uint32_t now_ms);
/** @brief 显式失效，保留上次数据和时间；不得与读取并发。 */
void BspRefereeData_Invalidate(void);

#endif /* SUPERPOWER_BSP_REFEREE_DATA_H */
