#include "prot_judge.h"
#include "string.h"
#include "crc.h"
#include "bsp_usart.h"
#include "string.h"

/* ==================== 比赛状态类数据 ==================== */
game_status_t                       game_status;                    //0x0001 比赛状态数据，固定以1Hz频率发送
game_result_t                       game_result;                    //0x0002 比赛结果数据，比赛结束触发发送
game_robot_HP_t                     game_robot_HP;                  //0x0003 机器人血量数据，固定以3Hz频率发送

/* ==================== 场地事件类数据 ==================== */
event_data_t                        event_data;                     //0x0101 场地事件数据，固定以1Hz频率发送
ext_supply_projectile_action_t      ext_supply_projectile_action;   //0x0102 补给站动作标识数据，补给站弹丸释放时触发发送
referee_warning_t                   referee_warning;                //0x0104 裁判警告数据，己方判罚/判负时触发发送，其余时间以1Hz频率发送
dart_info_t                         dart_info;                      //0x0105 飞镖发射相关数据，固定以1Hz频率发送

/* ==================== 机器人性能类数据 ==================== */
robot_status_t                      robot_status;                   //0x0201 机器人性能体系数据，固定以10Hz频率发送
power_heat_data_t                   power_heat_data;                //0x0202 实时底盘缓冲能量和射击热量数据，固定以10Hz频率发送
robot_pos_t                         robot_pos;                      //0x0203 机器人位置数据，固定以1Hz频率发送
buff_t                              buff;                           //0x0204 机器人增益和底盘能量数据，固定以3Hz频率发送
air_support_data_t                  air_support_data;               //0x0205 空中支援时间数据，固定以1Hz频率发送
hurt_data_t                         hurt_data;                      //0x0206 伤害状态数据，伤害发生后发送
shoot_data_t                        shoot_data;                     //0x0207 实时射击数据，弹丸发射后发送
projectile_allowance_t              projectile_allowance;           //0x0208 允许发弹量，固定以10Hz频率发送
rfid_status_t                       rfid_status;                    //0x0209 机器人RFID模块状态，固定以3Hz频率发送
dart_client_cmd_t                   dart_client_cmd;                //0x020A 飞镖选手端指令数据，固定以3Hz频率发送
ground_robot_position_t             ground_robot_position;          //0x020B 地面机器人位置数据，固定以1Hz频率发送
radar_mark_data_t                   radar_mark_data;                //0x020C 雷达标记进度数据，固定以1Hz频率发送
sentry_info_t                       sentry_info;                    //0x020D 哨兵自主决策信息同步，固定以1Hz频率发送
radar_info_t                        radar_info;                     //0x020E 雷达自主决策信息同步，固定以1Hz频率发送

/* ==================== 交互类数据 ==================== */
robot_interaction_data_t            robot_interaction_data;         //0x0301 机器人交互数据，发送方触发发送，频率上限为30Hz
custom_robot_data_t                 custom_robot_data;              //0x0302 自定义控制器与机器人交互数据 (图传链路)
map_command_t                       map_command;                    //0x0303 选手端小地图交互数据，选手端触发发送
// remote_control_t                 remote_control;                 //0x0304 【V1.2.0 已删除】键鼠遥控数据
map_robot_data_t                    map_robot_data;                 //0x0305 选手端小地图接收雷达数据，频率上限为10Hz
custom_client_data_t                custom_client_data;             //0x0306 自定义控制器与选手端交互数据
map_data_t                          map_data;                       //0x0307 选手端小地图接收哨兵数据，频率上限为1Hz
custom_info_t                       custom_info;                    //0x0308 选手端小地图接收机器人数据，频率上限为3Hz

/*
 * @brief     读取裁判数据函数，串口中断函数中直接调用进行读取
 * @param[in] data: 数据指针
 * @retval    数据正常返回0，异常返回1
 */
frame_header_t frame_header;
static void judge_copy_if_length(void *destination, const uint8_t *source, uint16_t actual_length, uint16_t expected_length)
{
    if (actual_length == expected_length)
    {
        memcpy(destination, source, expected_length);
    }
}

uint8_t judge_get_data(uint8_t *data, uint16_t data_len)
{
    uint16_t index = 0U;
    uint8_t result = 1U;
    uint16_t total_data_length;
    uint16_t cmd_id;

    while ((data != NULL) && ((uint16_t)(data_len - index) >= 9U))
    {
        if (data[index] != 0xA5U)
        {
            index++;
            continue;
        }

        memcpy(&frame_header, &data[index], sizeof(frame_header));
        if (!crc8_verify_checksum(&data[index], sizeof(frame_header)))
        {
            index++;
            continue;
        }

        total_data_length = (uint16_t)(frame_header.data_length + 9U);
        if ((total_data_length < 9U) || (total_data_length > (uint16_t)(data_len - index)))
        {
            break;
        }

        if (!crc16_verify_checksum(&data[index], total_data_length))
        {
            index++;
            continue;
        }

        result = 0U;
        cmd_id = (uint16_t)data[index + 5U] | ((uint16_t)data[index + 6U] << 8U);
        switch (cmd_id)
        {
            case ID_game_status:                  judge_copy_if_length(&game_status, &data[index + 7U], frame_header.data_length, LEN_game_status); break;
            case ID_game_result:                  judge_copy_if_length(&game_result, &data[index + 7U], frame_header.data_length, LEN_game_result); break;
            case ID_game_robot_HP:                judge_copy_if_length(&game_robot_HP, &data[index + 7U], frame_header.data_length, LEN_game_robot_HP); break;
            case ID_event_data:                   judge_copy_if_length(&event_data, &data[index + 7U], frame_header.data_length, LEN_event_data); break;
            case ID_ext_supply_projectile_action: judge_copy_if_length(&ext_supply_projectile_action, &data[index + 7U], frame_header.data_length, LEN_ext_supply_projectile_action); break;
            case ID_referee_warning:              judge_copy_if_length(&referee_warning, &data[index + 7U], frame_header.data_length, LEN_referee_warning); break;
            case ID_dart_info:                    judge_copy_if_length(&dart_info, &data[index + 7U], frame_header.data_length, LEN_dart_info); break;
            case ID_robot_status:                 judge_copy_if_length(&robot_status, &data[index + 7U], frame_header.data_length, LEN_robot_status); break;
            case ID_power_heat_data:              judge_copy_if_length(&power_heat_data, &data[index + 7U], frame_header.data_length, LEN_power_heat_data); break;
            case ID_robot_pos:                    judge_copy_if_length(&robot_pos, &data[index + 7U], frame_header.data_length, LEN_robot_pos); break;
            case ID_buff:                         judge_copy_if_length(&buff, &data[index + 7U], frame_header.data_length, LEN_buff); break;
            case ID_air_support_data:             judge_copy_if_length(&air_support_data, &data[index + 7U], frame_header.data_length, LEN_air_support_data); break;
            case ID_hurt_data:                    judge_copy_if_length(&hurt_data, &data[index + 7U], frame_header.data_length, LEN_hurt_data); break;
            case ID_shoot_data:                   judge_copy_if_length(&shoot_data, &data[index + 7U], frame_header.data_length, LEN_shoot_data); break;
            case ID_projectile_allowance:         judge_copy_if_length(&projectile_allowance, &data[index + 7U], frame_header.data_length, LEN_projectile_allowance); break;
            case ID_rfid_status:                  judge_copy_if_length(&rfid_status, &data[index + 7U], frame_header.data_length, LEN_rfid_status); break;
            case ID_dart_client_cmd:              judge_copy_if_length(&dart_client_cmd, &data[index + 7U], frame_header.data_length, LEN_dart_client_cmd); break;
            case ID_ground_robot_position:        judge_copy_if_length(&ground_robot_position, &data[index + 7U], frame_header.data_length, LEN_ground_robot_position); break;
            case ID_radar_mark_data:              judge_copy_if_length(&radar_mark_data, &data[index + 7U], frame_header.data_length, LEN_radar_mark_data); break;
            case ID_sentry_info:                  judge_copy_if_length(&sentry_info, &data[index + 7U], frame_header.data_length, LEN_sentry_info); break;
            case ID_radar_info:                   judge_copy_if_length(&radar_info, &data[index + 7U], frame_header.data_length, LEN_radar_info); break;
            case ID_robot_interaction_data:       judge_copy_if_length(&robot_interaction_data, &data[index + 7U], frame_header.data_length, LEN_robot_interaction_data); break;
            case ID_custom_robot_data:            judge_copy_if_length(&custom_robot_data, &data[index + 7U], frame_header.data_length, LEN_custom_robot_data); break;
            case ID_map_command:                  judge_copy_if_length(&map_command, &data[index + 7U], frame_header.data_length, LEN_map_command); break;
            case ID_map_robot_data:               judge_copy_if_length(&map_robot_data, &data[index + 7U], frame_header.data_length, LEN_map_robot_data); break;
            case ID_custom_client_data:           judge_copy_if_length(&custom_client_data, &data[index + 7U], frame_header.data_length, LEN_custom_client_data); break;
            case ID_map_data:                     judge_copy_if_length(&map_data, &data[index + 7U], frame_header.data_length, LEN_map_data); break;
            case ID_custom_info:                  judge_copy_if_length(&custom_info, &data[index + 7U], frame_header.data_length, LEN_custom_info); break;
            default: break;
        }

        index = (uint16_t)(index + total_data_length);
    }

    return result;
}
