/**
 * @file debug.c
 * @brief 适配旧工程 debug 模块，按档位观察本工程 PID 记录。
 */
#include "debug.h"

#include "data_scope.h"
#include "power_ctrl.h"

volatile DebugWave_e g_debug_wave = DEBUG_WAVE_CURRENT;

void DataWavePkg(void)
{
    // clang-format off
    switch(g_debug_wave)
    {
        case DEBUG_WAVE_CURRENT:
        {
            DataScope_Get_Channel_Data(g_power_ctrl.current_pid.Target);
            DataScope_Get_Channel_Data(g_power_ctrl.current_pid.Measure);
            DataScope_Get_Channel_Data(g_power_ctrl.current_pid.Output);
        }    break;

        case DEBUG_WAVE_CURRENT_VOLTAGE:
        {
            DataScope_Get_Channel_Data(g_power_ctrl.current_pid.Target);
            DataScope_Get_Channel_Data(g_power_ctrl.current_pid.Measure);
            DataScope_Get_Channel_Data(g_power_ctrl.current_pid.Output);
            DataScope_Get_Channel_Data(g_power_ctrl.voltage_pid.Target);
            DataScope_Get_Channel_Data(g_power_ctrl.voltage_pid.Measure);
            DataScope_Get_Channel_Data(g_power_ctrl.voltage_pid.Output);
        }    break;

        case DEBUG_WAVE_POWER:
        {
            DataScope_Get_Channel_Data(g_power_ctrl.power_pid.Target);
            DataScope_Get_Channel_Data(g_power_ctrl.power_pid.Measure);
            DataScope_Get_Channel_Data(g_power_ctrl.power_pid.Output);
        }    break;

        case DEBUG_WAVE_ENERGY:
        {
            DataScope_Get_Channel_Data(g_power_ctrl.energy_pid.Target);
            DataScope_Get_Channel_Data(g_power_ctrl.energy_pid.Measure);
            DataScope_Get_Channel_Data(g_power_ctrl.energy_pid.Output);
        }    break;

        case DEBUG_WAVE_OFF:
        default:
        {
            /* 无通道时不启动发送，已经进行的 DMA 不受影响。 */
        }    break;
    }
    // clang-format on
}
