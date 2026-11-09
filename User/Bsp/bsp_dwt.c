
/**
 * @file bsp_dwt.c
 * @brief STM32G4 DWT 执行时间测量模块实现。
 */

#include "bsp_dwt.h"

#include "stm32g4xx.h"
/* ==================== 公共变量 ==================== */

/**< 各计时槽执行耗时，单位 us。 */
volatile float g_dwt_us[DWT_SLOT_COUNT];

/* ==================== 私有变量 ==================== */

/**< 各计时槽起始 CPU 周期数。 */
static uint32_t s_start_cycles[DWT_SLOT_COUNT];

/**< CPU 单周期对应的微秒数。 */
static float s_us_per_cycle;

/* ==================== 公共实现 ==================== */

void Bsp_DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    s_us_per_cycle = 1000000.0f / (float)SystemCoreClock;
}

void Bsp_DWT_Start(uint32_t slot)
{
    s_start_cycles[slot] = DWT->CYCCNT;
}

void Bsp_DWT_Stop(uint32_t slot)
{
    g_dwt_us[slot] = (float)(DWT->CYCCNT - s_start_cycles[slot]) * s_us_per_cycle;
}
