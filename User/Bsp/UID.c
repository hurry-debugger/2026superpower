/**
 * @file UID.c
 * @brief 读取芯片 UID，匹配手填记录并发布全局 ADC 校准状态。
 */

#include "UID.h"

#include "adccalibration.h"
#include "stm32g4xx_hal.h"

/* ==================== 全局状态 ==================== */

AdcCalibrationState_t g_adc_calibration_state =
{
    .supercap_id = SUPERCAP_ID_UNKNOWN
};

/* ==================== 公共接口实现 ==================== */

UID_t UID_Read(void)
{
    UID_t uid =
    {
        .word = {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()}
    };

    return uid;
}

bool UID_Equals(const UID_t *a, const UID_t *b)
{
    return a->word[0] == b->word[0]
        && a->word[1] == b->word[1]
        && a->word[2] == b->word[2];
}

void UID_MatchCalibration(void)
{
    UID_t uid = UID_Read();
    SupercapId_e matched_id = SUPERCAP_ID_UNKNOWN;
    uint32_t match_count = 0U;

    for(uint32_t id = 0U; id < SUPERCAP_COUNT; id++)
    {
        const UID_t *record_uid = &g_adc_calibration_records[id].uid;

        /* 全零仅表示未登记槽位，不能被选为实际板卡记录。 */
        if((record_uid->word[0] | record_uid->word[1] | record_uid->word[2]) == 0U)
        {
            continue;
        }

        if(UID_Equals(&uid, record_uid))
        {
            matched_id = (SupercapId_e)id;
            match_count++;
        }
    }

    if(match_count != 1U)
    {
        matched_id = SUPERCAP_ID_UNKNOWN;
    }

    AdcCalibrationState_t state = AdcCalibration_Init(matched_id);
    state.uid = uid;

    if(match_count > 1U)
    {
        /* 保留理论系数，但区分无记录和重复匹配，避免误选其中一条。 */
        state.board_errors = ADC_CAL_BOARD_ERR_DUPLICATE_UID;
    }

    g_adc_calibration_state = state;
}
