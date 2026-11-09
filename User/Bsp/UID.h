/**
 * @file UID.h
 * @brief 芯片完整 UID 的读取与比较底层接口。
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
 * @brief 读取当前 MCU 的完整 UID，直接填写调用方提供的对象。
 * @param[out] uid 接收 96 位 UID，按 HAL 的 w0、w1、w2 顺序写入三个字。
 * @pre 指针非空，指向可写的 UID_t 对象。
 * @note 在启动主线程调用；不保存额外副本，不执行外设配置或校准。
 */
void UID_Read(UID_t *uid);

/**
 * @brief 比较两个完整 UID。
 * @param[in] a 第一个 UID，调用期间保持稳定。
 * @param[in] b 第二个 UID，调用期间保持稳定。
 * @return 三个字全部相等时为 true，否则为 false。
 * @pre 两个指针均非空。
 */
bool UID_Equals(const UID_t *a, const UID_t *b);

#ifdef __cplusplus
}
#endif

#endif /* SUPERPOWER_UID_H */
