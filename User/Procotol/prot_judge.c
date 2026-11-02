/**
 * @file prot_judge.c
 * @brief 裁判协议有界解帧、CRC 校验和数据接收状态发布。
 * @note 源自 Taurus；不访问 HAL、UART 或 RTOS，不负责跨批次半帧拼接。
 */
#include "prot_judge.h"

#include "crc.h"

#include <string.h>

/* ==================== 私有定义 ==================== */

/** @brief 源协议帧头 5 字节、命令号 2 字节、CRC16 2 字节。 */
#define JUDGE_HEADER_LENGTH (5U)
#define JUDGE_PAYLOAD_OFFSET (7U)
#define JUDGE_FRAME_OVERHEAD (9U)
#define JUDGE_FRAME_SOF (0xa5U)

/* ==================== 比赛状态类数据 ==================== */
game_status_t game_status; /**< 0x0001 比赛状态数据，固定以1Hz频率发送 */
game_result_t game_result; /**< 0x0002 比赛结果数据，比赛结束触发发送 */
game_robot_HP_t game_robot_HP; /**< 0x0003 机器人血量数据，固定以3Hz频率发送 */

/* ==================== 场地事件类数据 ==================== */
event_data_t event_data; /**< 0x0101 场地事件数据，固定以1Hz频率发送 */
ext_supply_projectile_action_t ext_supply_projectile_action; /**< 0x0102 补给站动作标识数据，补给站弹丸释放时触发发送 */
referee_warning_t referee_warning; /**< 0x0104 裁判警告数据，己方判罚/判负时触发发送，其余时间以1Hz频率发送 */
dart_info_t dart_info; /**< 0x0105 飞镖发射相关数据，固定以1Hz频率发送 */

/* ==================== 机器人性能类数据 ==================== */
robot_status_t robot_status; /**< 0x0201 机器人性能体系数据，固定以10Hz频率发送 */
power_heat_data_t power_heat_data; /**< 0x0202 实时底盘缓冲能量和射击热量数据，固定以10Hz频率发送 */
robot_pos_t robot_pos; /**< 0x0203 机器人位置数据，固定以1Hz频率发送 */
buff_t buff; /**< 0x0204 机器人增益和底盘能量数据，固定以3Hz频率发送 */
air_support_data_t air_support_data; /**< 0x0205 空中支援时间数据，固定以1Hz频率发送 */
hurt_data_t hurt_data; /**< 0x0206 伤害状态数据，伤害发生后发送 */
shoot_data_t shoot_data; /**< 0x0207 实时射击数据，弹丸发射后发送 */
projectile_allowance_t projectile_allowance; /**< 0x0208 允许发弹量，固定以10Hz频率发送 */
rfid_status_t rfid_status; /**< 0x0209 机器人RFID模块状态，固定以3Hz频率发送 */
dart_client_cmd_t dart_client_cmd; /**< 0x020A 飞镖选手端指令数据，固定以3Hz频率发送 */
ground_robot_position_t ground_robot_position; /**< 0x020B 地面机器人位置数据，固定以1Hz频率发送 */
radar_mark_data_t radar_mark_data; /**< 0x020C 雷达标记进度数据，固定以1Hz频率发送 */
sentry_info_t sentry_info; /**< 0x020D 哨兵自主决策信息同步，固定以1Hz频率发送 */
radar_info_t radar_info; /**< 0x020E 雷达自主决策信息同步，固定以1Hz频率发送 */

/* ==================== 交互类数据 ==================== */
robot_interaction_data_t robot_interaction_data; /**< 0x0301 机器人交互数据，发送方触发发送，频率上限为30Hz */
custom_robot_data_t custom_robot_data; /**< 0x0302 自定义控制器与机器人交互数据 (图传链路) */
map_command_t map_command; /**< 0x0303 选手端小地图交互数据，选手端触发发送 */
map_robot_data_t map_robot_data; /**< 0x0305 选手端小地图接收雷达数据，频率上限为10Hz */
custom_client_data_t custom_client_data; /**< 0x0306 自定义控制器与选手端交互数据 */
map_data_t map_data; /**< 0x0307 选手端小地图接收哨兵数据，频率上限为1Hz */
custom_info_t custom_info; /**< 0x0308 选手端小地图接收机器人数据，频率上限为3Hz */

frame_header_t frame_header;
JudgeRxState_t g_judge_robot_status_rx;
JudgeRxState_t g_judge_power_heat_rx;

/* ==================== 公共接口 ==================== */

uint8_t judge_get_data(uint8_t *data, uint16_t rx_len, uint32_t now_ms)
{
    uint8_t result = 1U;
    uint32_t offset = 0U;

    while((uint32_t)rx_len - offset >= JUDGE_FRAME_OVERHEAD)
    {
        uint8_t *frame = &data[offset];
        uint32_t remaining = (uint32_t)rx_len - offset;
        if(frame[0] != JUDGE_FRAME_SOF
           || !crc8_verify_checksum(frame, JUDGE_HEADER_LENGTH))
        {
            offset++;
            continue;
        }

        /* 先提升至 32 位再加固定开销，避免伪造长度导致 16 位回卷。 */
        uint16_t payload_length = (uint16_t)((uint16_t)frame[1] | ((uint16_t)frame[2] << 8));
        uint32_t frame_length = (uint32_t)payload_length + JUDGE_FRAME_OVERHEAD;
        if(frame_length > remaining)
        {
            /* 当前候选帧不完整；继续搜索后续完整帧，不保存半帧。 */
            offset++;
            continue;
        }
        if(!crc16_verify_checksum(frame, (uint16_t)frame_length))
        {
            offset++;
            continue;
        }

        uint16_t cmd_id = (uint16_t)((uint16_t)frame[5] | ((uint16_t)frame[6] << 8));
        void *destination = NULL;
        uint16_t expected_length = 0U;
        uint32_t destination_size = 0U;
        JudgeRxState_t *rx_state = NULL;

        // clang-format off
        switch(cmd_id)
        {
            case ID_game_status:
            {
                destination = &game_status;
                expected_length = LEN_game_status;
                destination_size = sizeof(game_status);
            }    break;

            case ID_game_result:
            {
                destination = &game_result;
                expected_length = LEN_game_result;
                destination_size = sizeof(game_result);
            }    break;

            case ID_game_robot_HP:
            {
                destination = &game_robot_HP;
                expected_length = LEN_game_robot_HP;
                destination_size = sizeof(game_robot_HP);
            }    break;

            case ID_event_data:
            {
                destination = &event_data;
                expected_length = LEN_event_data;
                destination_size = sizeof(event_data);
            }    break;

            case ID_ext_supply_projectile_action:
            {
                destination = &ext_supply_projectile_action;
                expected_length = LEN_ext_supply_projectile_action;
                destination_size = sizeof(ext_supply_projectile_action);
            }    break;

            case ID_referee_warning:
            {
                destination = &referee_warning;
                expected_length = LEN_referee_warning;
                destination_size = sizeof(referee_warning);
            }    break;

            case ID_dart_info:
            {
                destination = &dart_info;
                expected_length = LEN_dart_info;
                destination_size = sizeof(dart_info);
            }    break;

            case ID_robot_status:
            {
                destination = &robot_status;
                expected_length = LEN_robot_status;
                destination_size = sizeof(robot_status);
                rx_state = &g_judge_robot_status_rx;
            }    break;

            case ID_power_heat_data:
            {
                destination = &power_heat_data;
                expected_length = LEN_power_heat_data;
                destination_size = sizeof(power_heat_data);
                rx_state = &g_judge_power_heat_rx;
            }    break;

            case ID_robot_pos:
            {
                destination = &robot_pos;
                expected_length = LEN_robot_pos;
                destination_size = sizeof(robot_pos);
            }    break;

            case ID_buff:
            {
                destination = &buff;
                expected_length = LEN_buff;
                destination_size = sizeof(buff);
            }    break;

            case ID_air_support_data:
            {
                destination = &air_support_data;
                expected_length = LEN_air_support_data;
                destination_size = sizeof(air_support_data);
            }    break;

            case ID_hurt_data:
            {
                destination = &hurt_data;
                expected_length = LEN_hurt_data;
                destination_size = sizeof(hurt_data);
            }    break;

            case ID_shoot_data:
            {
                destination = &shoot_data;
                expected_length = LEN_shoot_data;
                destination_size = sizeof(shoot_data);
            }    break;

            case ID_projectile_allowance:
            {
                destination = &projectile_allowance;
                expected_length = LEN_projectile_allowance;
                destination_size = sizeof(projectile_allowance);
            }    break;

            case ID_rfid_status:
            {
                destination = &rfid_status;
                expected_length = LEN_rfid_status;
                destination_size = sizeof(rfid_status);
            }    break;

            case ID_dart_client_cmd:
            {
                destination = &dart_client_cmd;
                expected_length = LEN_dart_client_cmd;
                destination_size = sizeof(dart_client_cmd);
            }    break;

            case ID_ground_robot_position:
            {
                destination = &ground_robot_position;
                expected_length = LEN_ground_robot_position;
                destination_size = sizeof(ground_robot_position);
            }    break;

            case ID_radar_mark_data:
            {
                destination = &radar_mark_data;
                expected_length = LEN_radar_mark_data;
                destination_size = sizeof(radar_mark_data);
            }    break;

            case ID_sentry_info:
            {
                destination = &sentry_info;
                expected_length = LEN_sentry_info;
                destination_size = sizeof(sentry_info);
            }    break;

            case ID_radar_info:
            {
                destination = &radar_info;
                expected_length = LEN_radar_info;
                destination_size = sizeof(radar_info);
            }    break;

            case ID_robot_interaction_data:
            {
                destination = &robot_interaction_data;
                expected_length = LEN_robot_interaction_data;
                destination_size = sizeof(robot_interaction_data);
            }    break;

            case ID_custom_robot_data:
            {
                destination = &custom_robot_data;
                expected_length = LEN_custom_robot_data;
                destination_size = sizeof(custom_robot_data);
            }    break;

            case ID_map_command:
            {
                destination = &map_command;
                expected_length = LEN_map_command;
                destination_size = sizeof(map_command);
            }    break;

            case ID_map_robot_data:
            {
                destination = &map_robot_data;
                expected_length = LEN_map_robot_data;
                destination_size = sizeof(map_robot_data);
            }    break;

            case ID_custom_client_data:
            {
                destination = &custom_client_data;
                expected_length = LEN_custom_client_data;
                destination_size = sizeof(custom_client_data);
            }    break;

            case ID_map_data:
            {
                destination = &map_data;
                expected_length = LEN_map_data;
                destination_size = sizeof(map_data);
            }    break;

            case ID_custom_info:
            {
                destination = &custom_info;
                expected_length = LEN_custom_info;
                destination_size = sizeof(custom_info);
            }    break;

            default:
            {
                /* 未支持命令跳过整帧，不将载荷内容作为新帧解析。 */
            }    break;
        }
        // clang-format on

        /* 同时核对协议长度及对象大小，不为源布局差异执行越界复制。 */
        if(destination != NULL && payload_length == expected_length
           && destination_size == expected_length)
        {
            memcpy(destination, frame + JUDGE_PAYLOAD_OFFSET, expected_length);
            memcpy(&frame_header, frame, JUDGE_HEADER_LENGTH);
            if(rx_state != NULL)
            {
                rx_state->update_ms = now_ms;
                rx_state->valid = true;
            }
            result = 0U;
        }
        offset += frame_length;
    }
    return result;
}
