/** @file bsp_referee_data.c
 * @brief 裁判解析后数值及最后有效时间的发布实现。
 */
#include "bsp_referee_data.h"

#include <math.h>

BspRefereeData_t g_bsp_referee_data;

void BspRefereeData_Publish(float power_limit_w, float buffer_energy_j, uint32_t now_ms)
{
    if(!isfinite(power_limit_w) || power_limit_w < 0.0f
       || !isfinite(buffer_energy_j) || buffer_energy_j < 0.0f)
    {
        BspRefereeData_Invalidate();
        return;
    }
    g_bsp_referee_data.valid = false;
    g_bsp_referee_data.power_limit_w = power_limit_w;
    g_bsp_referee_data.buffer_energy_j = buffer_energy_j;
    g_bsp_referee_data.update_ms = now_ms;
    g_bsp_referee_data.valid = true;
}

void BspRefereeData_Invalidate(void)
{
    g_bsp_referee_data.valid = false;
}
