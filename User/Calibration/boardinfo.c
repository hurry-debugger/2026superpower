/**
 * @file boardinfo.c
 * @brief 管理板卡主要信息，组织底层 UID 识别与 ADC 启动校准。
 */
#include "boardinfo.h"

#include <string.h>

/* ==================== 全局状态 ==================== */

BoardInfo_t g_board_info;

/* ==================== 公共接口实现 ==================== */

void BoardInfo_Init(void)
{
    memset(&g_board_info, 0, sizeof(g_board_info));
    UID_Read(&g_board_info.uid);
    g_board_info.supercap_id = SUPERCAP_ID_UNKNOWN;
    uint32_t match_count = 0U;

    for(uint32_t id = SUPERCAP_ID_01; id < SUPERCAP_COUNT; id++)
    {
        /* 全零槽位尚未登记；完整 UID 相同才计为一次匹配。 */
        if((g_adc_calibration_records[id].uid.word[0]
            | g_adc_calibration_records[id].uid.word[1]
            | g_adc_calibration_records[id].uid.word[2]) != 0U
           && UID_Equals(&g_board_info.uid, &g_adc_calibration_records[id].uid))
        {
            g_board_info.supercap_id = (SupercapId_e)id;
            match_count++;
        }
    }

    if(match_count == 1U)
    {
        for(uint32_t channel = 0U; channel < ADC_CHANNEL_COUNT; channel++)
        {
            g_board_info.channel_errors[channel] = AdcCalibration_FitChannel(
                g_board_info.supercap_id, (AdcChannel_e)channel);

            if(!g_board_info.channel_errors[channel].count_range
               && !g_board_info.channel_errors[channel].zero_variance
               && !g_board_info.channel_errors[channel].gain_nonpositive
               && !g_board_info.channel_errors[channel].numeric_range
               && !g_board_info.channel_errors[channel].value_range)
            {
                g_board_info.valid_channel_mask |= 1U << channel;
            }
        }
    }
    else
    {
        g_board_info.supercap_id = SUPERCAP_ID_UNKNOWN;
        g_board_info.board_errors.no_record = match_count == 0U;
        g_board_info.board_errors.duplicate_uid = match_count > 1U;
    }
}
