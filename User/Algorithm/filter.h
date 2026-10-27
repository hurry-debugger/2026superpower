/**
 * @file filter.h
 * @brief 原始计数平均与物理量一阶低通接口。
 * @details 先对多次整数采样求平均，再换算物理量，最后按需调用浮点低通。
 * @note 不访问外设、不持有共享状态；可在主线程或中断中使用，执行时间需在目标板测量。
 */

#ifndef SUPERPOWER_FILTER_H
#define SUPERPOWER_FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ==================== 公共接口 ==================== */

/**
 * @brief 对等间隔存放的无符号半字采样求算术平均，四舍五入为整数。
 * @param[in] samples 第一个待处理样本的地址；各样本取值范围为 0～65535。
 * @param[in] sample_count 样本数量，范围为 1～65535。
 * @param[in] sample_stride 相邻待处理样本间隔的 uint16_t 元素数，范围为 1～65535。
 * @return 平均计数，范围为 0～65535；不足一个计数的小数在此舍入。
 * @pre samples 非空，至少可读取 (sample_count - 1) * sample_stride + 1 个元素，
 *      且整个调用期间输入保持稳定；索引表达式和内存范围须在目标平台可表示。
 * @note ADC1 四通道交错数据的步长为 4，ADC2 单通道步长为 1。
 *       使用 uint32_t 累加；不修改输入、不检查参数、不返回函数状态。
 */
uint16_t Filter_AverageU16(const uint16_t *samples, uint16_t sample_count, uint16_t sample_stride);

/**
 * @brief 对已换算的物理量执行一次一阶低通更新。
 * @param[in] input 本次物理量，例如 V 或 A；须与 previous 使用相同单位。
 * @param[in] previous 上一次滤波结果；首次调用前由调用方用当前 input 初始化。
 * @param[in] alpha 无量纲更新系数，范围为 [0, 1]；越大越接近本次输入。
 * @return previous + alpha * (input - previous)，单位与输入相同。
 * @pre 三个参数均有限，差值及运算结果可由 float 表示。
 * @note alpha 为 0 时保持历史，为 1 时跟随输入；实际浮点结果受舍入影响。
 *       调用方分别保存各通道历史，并在固定反馈周期更新；周期变化时需重新选择系数。
 *       不持有状态、不检查参数；static inline 不保证编译器一定展开调用。
 */
static inline float Filter_LowPass(float input, float previous, float alpha)
{
    return previous + alpha * (input - previous);
}

#ifdef __cplusplus
}
#endif

#endif /* SUPERPOWER_FILTER_H */
