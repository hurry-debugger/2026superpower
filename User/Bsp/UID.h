/**
 * @file UID.h
 * @brief 芯片完整 UID 的读取、比较及启动校准匹配入口。
 * @note 文件名 UID 为用户指定例外；全部函数实现在 UID.c。
 */

#ifndef SUPERPOWER_UID_H
#define SUPERPOWER_UID_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief 完整 96 位 UID；字序固定为 HAL_GetUIDw0、w1、w2。 */
typedef struct
{
    uint32_t word[3]; /**< 三个 32 位字，无单位，不截断或哈希。 */
} UID_t;

/**
 * @brief 读取当前 MCU 的完整 UID。
 * @return 按 w0、w1、w2 顺序保存的 UID，可直接用于填写校准记录。
 * @note 在启动主线程中调用，不执行外设配置或校准。
 */
UID_t UID_Read(void);

/**
 * @brief 比较两个完整 UID。
 * @param[in] a 第一个 UID，调用期间保持稳定。
 * @param[in] b 第二个 UID，调用期间保持稳定。
 * @return 三个字全部相等时为 true，否则为 false。
 * @pre 两个指针均非空。
 */
bool UID_Equals(const UID_t *a, const UID_t *b);

/**
 * @brief 读取一次 UID、唯一匹配记录并初始化全局校准状态及换算系数。
 * @pre 仅在启动主线程中调用，功率输出关闭，换算消费者尚未运行。
 * @note 跳过全零 UID 槽位；未知或重复 UID 恢复理论换算，但禁止控制。
 *       g_adc_calibration_state 的唯一运行期更新者是本函数；发布后其他模块只读。
 *       该函数不启动或关闭 HRTIM，输出启动门槛由 main 检查。
 */
void UID_MatchCalibration(void);

#ifdef __cplusplus
}
#endif

#endif /* SUPERPOWER_UID_H */
