/**
 * @file debug.h
 * @brief 四环调试波形档位与数据打包接口。
 */
#ifndef SUPERPOWER_DEBUG_H
#define SUPERPOWER_DEBUG_H

/** @brief 仅选择观察数据，不切换控制环或功率输出模式。 */
typedef enum
{
    DEBUG_WAVE_OFF = 0, /**< 停止启动新帧，已启动的 DMA 自然发送完成。 */
    DEBUG_WAVE_CURRENT = 1, /**< 电流目标 A、反馈 A、无量纲输出修正。 */
    DEBUG_WAVE_CURRENT_VOLTAGE = 2, /**< 电流三通道后追加电压目标 V、反馈 V、输出修正。 */
    DEBUG_WAVE_POWER = 3, /**< 独立打印功率目标 W、反馈 W、输出电流 A。 */
    DEBUG_WAVE_ENERGY = 4 /**< 独立打印能量目标 J、反馈 J、输出功率修正 W。 */
} DebugWave_e;

/** @brief 调试器可修改的打印档位，默认电流环；非法值不发送新帧。 */
extern volatile DebugWave_e g_debug_wave;

/**
 * @brief 按当前档位将 PID 的 Target、Measure、Output 顺序加入本帧。
 * @pre 仅由 DataWave 在保存 PRIMASK 的短临界区内调用，发送缓冲区已空闲。
 * @note 保护状态下 PID 可能保留旧值；输出修正不是最终广义占空比。
 */
void DataWavePkg(void);

#endif /* SUPERPOWER_DEBUG_H */
