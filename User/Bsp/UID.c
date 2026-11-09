/**
 * @file UID.c
 * @brief 读取和比较芯片完整 UID，不依赖板卡信息或 ADC 校准模块。
 */
#include "UID.h"

#include "stm32g4xx_hal.h"

/* ==================== 公共接口实现 ==================== */

void UID_Read(UID_t *uid)
{
    uid->word[0] = HAL_GetUIDw0();
    uid->word[1] = HAL_GetUIDw1();
    uid->word[2] = HAL_GetUIDw2();
}

bool UID_Equals(const UID_t *a, const UID_t *b)
{
    return a->word[0] == b->word[0]
        && a->word[1] == b->word[1]
        && a->word[2] == b->word[2];
}
