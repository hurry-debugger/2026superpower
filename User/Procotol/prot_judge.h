/**
 * @file prot_judge.h
 * @brief Taurus 裁判报文类型、接收数据及有界解析接口。
 * @note 线上字段、命令号和长度沿用源工程。仅线上结构体使用 pack(1)；
 *       假定 STM32 小端及源编译器位域布局，目标工具链兼容性待验证。
 */
#ifndef SUPERPOWER_PROT_JUDGE_H
#define SUPERPOWER_PROT_JUDGE_H

#include <stdbool.h>
#include <stdint.h>

/** @brief 单次 DMA 接收容量，沿用源工程 150 字节。 */
#define JUDGE_DATA_LEN (150U)

#pragma pack(1)
/** @brief 帧头 */
typedef struct
{
    uint8_t SOF; /**< 数据帧起始字节，固定值为 0xA5 */
    uint16_t data_length; /**< 数据帧中 data 的长度 */
    uint8_t seq; /**< 包序号 */
    uint8_t CRC8; /**< 帧头 CRC8 校验 */
} frame_header_t;
#pragma pack()
/** @brief 命令码 */
typedef enum
{
    ID_game_status = 0x0001, /**< 比赛状态数据，固定以1Hz频率发送 */
    ID_game_result = 0x0002, /**< 比赛结果数据，比赛结束触发发送 */
    ID_game_robot_HP = 0x0003, /**< 机器人血量数据，固定以3Hz频率发送 */
    ID_event_data = 0x0101, /**< 场地事件数据，固定以1Hz频率发送 */
    ID_ext_supply_projectile_action = 0x0102, /**< 补给站动作标识数据，补给站弹丸释放时触发发送 */
    ID_referee_warning = 0x0104, /**< 裁判警告数据，己方判罚/判负时触发发送，其余时间以1Hz频率发送 */
    ID_dart_info = 0x0105, /**< 飞镖发射相关数据，固定以1Hz频率发送 */

    ID_robot_status = 0x0201, /**< 机器人性能体系数据，固定以10Hz频率发送 */
    ID_power_heat_data = 0x0202, /**< 实时底盘缓冲能量和射击热量数据，固定以10Hz频率发送 */
    ID_robot_pos = 0x0203, /**< 机器人位置数据，固定以1Hz频率发送 */
    ID_buff = 0x0204, /**< 机器人增益和底盘能量数据，固定以3Hz频率发送 */
    ID_air_support_data = 0x0205, /**< 空中支援时间数据，固定以1Hz频率发送 */
    ID_hurt_data = 0x0206, /**< 伤害状态数据，伤害发生后发送 */
    ID_shoot_data = 0x0207, /**< 实时射击数据，弹丸发射后发送 */
    ID_projectile_allowance = 0x0208, /**< 允许发弹量，固定以10Hz频率发送 */
    ID_rfid_status = 0x0209, /**< 机器人RFID模块状态，固定以3Hz频率发送 */
    ID_dart_client_cmd = 0x020A, /**< 飞镖选手端指令数据，固定以3Hz频率发送 */
    ID_ground_robot_position = 0x020B, /**< 地面机器人位置数据，固定以1Hz频率发送 */
    ID_radar_mark_data = 0x020C, /**< 雷达标记进度数据，固定以1Hz频率发送 */
    ID_sentry_info = 0x020D, /**< 哨兵自主决策信息同步，固定以1Hz频率发送 */
    ID_radar_info = 0x020E, /**< 雷达自主决策信息同步，固定以1Hz频率发送 */

    ID_robot_interaction_data = 0x0301, /**< 机器人交互数据，发送方触发发送，频率上限为30Hz */
    ID_custom_robot_data = 0x0302, /**< 自定义控制器与机器人交互数据，发送方触发发送，频率上限为30Hz        图传链路 */
    ID_map_command = 0x0303, /**< 选手端小地图交互数据，选手端触发发送 */
    ID_map_robot_data = 0x0305, /**< 选手端小地图接收雷达数据，频率上限为5Hz */
    ID_custom_client_data = 0x0306, /**< 自定义控制器与选手端交互数据，发送方触发发送，频率上限为30Hz */
    ID_map_data = 0x0307, /**< 选手端小地图接收路径数据，频率上限为1Hz */
    ID_custom_info = 0x0308 /**< 选手端小地图接收机器人数据，频率上限为3Hz */
} cmd_id_e;

/** @brief 数据段长度 (根据 RoboMaster 2026 协议 V1.2.0 详细定义) */
typedef enum
{
    LEN_game_status = 11, /**< 0x0001 */
    LEN_game_result = 1, /**< 0x0002 */
    LEN_game_robot_HP = 16, /**< 0x0003 【V1.2.0 改为己方血量，16字节】 */

    LEN_event_data = 4, /**< 0x0101 */
    LEN_ext_supply_projectile_action = 4, /**< 0x0102 */
    LEN_referee_warning = 3, /**< 0x0104 */
    LEN_dart_info = 3, /**< 0x0105 */

    LEN_robot_status = 13, /**< 0x0201 */
    LEN_power_heat_data = 14, /**< 0x0202 【V1.2.0 改为14字节】 */
    LEN_robot_pos = 12, /**< 0x0203 【V1.2.0 无z坐标，12字节】 */
    LEN_buff = 8, /**< 0x0204 【V1.2.0 新增remaining_energy，8字节】 */
    LEN_air_support_data = 2, /**< 0x0205 */
    LEN_hurt_data = 1, /**< 0x0206 */
    LEN_shoot_data = 7, /**< 0x0207 */
    LEN_projectile_allowance = 8, /**< 0x0208 【V1.2.0 新增fortress，8字节】 */
    LEN_rfid_status = 5, /**< 0x0209 【V1.2.0 新增rfid_status_2，5字节】 */
    LEN_dart_client_cmd = 6, /**< 0x020A */
    LEN_ground_robot_position = 40, /**< 0x020B */
    LEN_radar_mark_data = 2, /**< 0x020C */
    LEN_sentry_info = 6, /**< 0x020D 【V1.2.0 新增sentry_info_2，6字节】 */
    LEN_radar_info = 1, /**< 0x020E */

    LEN_robot_interaction_data = 118, /**< 0x0301 【V1.2.0 改为118字节】 */
    LEN_custom_robot_data = 30, /**< 0x0302 */
    LEN_map_command = 12, /**< 0x0303 【V1.2.0 确认为12字节】 */
    LEN_map_robot_data = 24, /**< 0x0305 */
    LEN_custom_client_data = 8, /**< 0x0306 */
    LEN_map_data = 103, /**< 0x0307 */
    LEN_custom_info = 34 /**< 0x0308 */
} cmd_len_e;

/** @brief 各数据结构体定义 */
#pragma pack(1)
typedef struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_status_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
    uint8_t winner;
} game_result_t;
#pragma pack()
#pragma pack(1)
/** @brief 0x0003 机器人血量数据 (16字节) 【V1.2.0 改为只发送己方血量】 */
typedef struct
{
    uint16_t ally_1_robot_HP; /**< 己方1号英雄机器人血量 */
    uint16_t ally_2_robot_HP; /**< 己方2号工程机器人血量 */
    uint16_t ally_3_robot_HP; /**< 己方3号步兵机器人血量 */
    uint16_t ally_4_robot_HP; /**< 己方4号步兵机器人血量 */
    uint16_t reserved; /**< 保留位 */
    uint16_t ally_7_robot_HP; /**< 己方7号哨兵机器人血量 */
    uint16_t ally_outpost_HP; /**< 己方前哨站血量 */
    uint16_t ally_base_HP; /**< 己方基地血量 */
} game_robot_HP_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
    uint32_t event_data;
} event_data_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
    uint8_t reserved;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} referee_warning_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} dart_info_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} robot_status_t;
#pragma pack()
#pragma pack(1)
/** @brief 0x0202 实时底盘缓冲能量和射击热量数据 (14字节) 【V1.2.0 简化】 */
typedef struct
{
    uint16_t reserved1; /**< 保留位 */
    uint16_t reserved2; /**< 保留位 */
    float reserved3; /**< 保留位 */
    uint16_t buffer_energy; /**< 缓冲能量（单位：J） */
    uint16_t shooter_17mm_barrel_heat; /**< 17mm发射机构的射击热量 */
    uint16_t shooter_42mm_barrel_heat; /**< 42mm发射机构的射击热量 */
} power_heat_data_t;
#pragma pack()
#pragma pack(1)
/** @brief 0x0203 机器人位置数据 (12字节) 【V1.2.0 删除z坐标，yaw改为angle】 */
typedef struct
{
    float x; /**< 位置x坐标，单位m */
    float y; /**< 位置y坐标，单位m */
    float angle; /**< 测速模块的朝向，单位°，正北为0度 */
} robot_pos_t;
#pragma pack()
#pragma pack(1)
/** @brief 0x0204 机器人增益和底盘能量数据 (8字节) 【V1.2.0 cooling_buff改为uint16_t，新增remaining_energy】 */
typedef struct
{
    uint8_t recovery_buff; /**< 回血增益（百分比） */
    uint16_t cooling_buff; /**< 射击热量冷却增益具体值（直接值） */
    uint8_t defence_buff; /**< 防御增益（百分比） */
    uint8_t vulnerability_buff; /**< 负防御增益（百分比） */
    uint16_t attack_buff; /**< 攻击增益（百分比） */
    uint8_t remaining_energy; /**< 机器人剩余能量值反馈位域 */
} buff_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
    uint8_t airforce_status;
    uint8_t time_remain;
} air_support_data_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
} hurt_data_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
} shoot_data_t;
#pragma pack()
#pragma pack(1)
/** @brief 0x0208 允许发弹量 (8字节) 【V1.2.0 新增堡垒增益点发弹量】 */
typedef struct
{
    uint16_t projectile_allowance_17mm; /**< 17mm弹丸允许发弹量 */
    uint16_t projectile_allowance_42mm; /**< 42mm弹丸允许发弹量 */
    uint16_t remaining_gold_coin; /**< 剩余金币数量 */
    uint16_t projectile_allowance_fortress; /**< 堡垒增益点提供的储备17mm弹丸允许发弹量 */
} projectile_allowance_t;
#pragma pack()
#pragma pack(1)
/** @brief 0x0209 机器人RFID模块状态 (5字节) 【V1.2.0 新增rfid_status_2】 */
typedef struct
{
    uint32_t rfid_status; /**< RFID状态位域 (bit 0-31) */
    uint8_t rfid_status_2; /**< RFID状态位域扩展 (bit 0-5为隧道相关) */
} rfid_status_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;
#pragma pack()
#pragma pack(1)
/** @brief 0x020B 地面机器人位置数据 (40字节) 【V1.2.0 5号改为保留位】 */
typedef struct
{
    float hero_x; /**< 己方英雄机器人位置x轴坐标，单位：m */
    float hero_y; /**< 己方英雄机器人位置y轴坐标，单位：m */
    float engineer_x; /**< 己方工程机器人位置x轴坐标，单位：m */
    float engineer_y; /**< 己方工程机器人位置y轴坐标，单位：m */
    float standard_3_x; /**< 己方3号步兵机器人位置x轴坐标，单位：m */
    float standard_3_y; /**< 己方3号步兵机器人位置y轴坐标，单位：m */
    float standard_4_x; /**< 己方4号步兵机器人位置x轴坐标，单位：m */
    float standard_4_y; /**< 己方4号步兵机器人位置y轴坐标，单位：m */
    float reserved_x; /**< 保留位 */
    float reserved_y; /**< 保留位 */
} ground_robot_position_t;
#pragma pack()
#pragma pack(1)
/** @brief 0x020C 雷达标记进度数据 (2字节，位域) */
typedef struct
{
    uint16_t mark_progress; /**< 标记进度位域 */
    /** @brief bit 0: 英雄机器人标记进度 */
    /** @brief bit 1: 工程机器人标记进度 */
    /** @brief bit 2: 步兵3号机器人标记进度 */
    /** @brief bit 3: 步兵4号机器人标记进度 */
    /** @brief bit 4: 步兵5号机器人标记进度 */
    /** @brief bit 5: 哨兵机器人标记进度 */
} radar_mark_data_t;
#pragma pack()
#pragma pack(1)
/** @brief 0x020D 哨兵自主决策信息同步 (6字节) 【V1.2.0 新增sentry_info_2】 */
typedef struct
{
    uint32_t sentry_info; /**< 哨兵信息位域 (bit 0-31) */
    uint16_t sentry_info_2; /**< 哨兵信息扩展 (bit 0-15) */
} sentry_info_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
    uint8_t radar_info;
} radar_info_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
    uint8_t data[30];
} custom_robot_data_t;
#pragma pack()
#pragma pack(1)
/** @brief 0x0303 选手端小地图交互数据 (12字节) 【V1.2.0 新增cmd_source】 */
typedef struct
{
    float target_position_x; /**< 目标位置x坐标 */
    float target_position_y; /**< 目标位置y坐标 */
    uint8_t cmd_keyboard; /**< 发送指令时的键盘按键通用键值 */
    uint8_t target_robot_id; /**< 目标机器人ID（发送坐标时为0） */
    uint16_t cmd_source; /**< 信息来源ID */
} map_command_t;
#pragma pack()

/** @brief 【V1.2.0 已删除】0x0304 remote_control_t 键鼠遥控数据结构体 */

#pragma pack(1)
/** @brief 0x0305 选手端小地图接收雷达数据 (24字节) 【V1.2.0 改为坐标格式】 */
typedef struct
{
    uint16_t hero_position_x; /**< 英雄机器人x位置坐标，单位：cm */
    uint16_t hero_position_y; /**< 英雄机器人y位置坐标，单位：cm */
    uint16_t engineer_position_x; /**< 工程机器人x位置坐标，单位：cm */
    uint16_t engineer_position_y; /**< 工程机器人y位置坐标，单位：cm */
    uint16_t infantry_3_position_x; /**< 3号步兵机器人x位置坐标，单位：cm */
    uint16_t infantry_3_position_y; /**< 3号步兵机器人y位置坐标，单位：cm */
    uint16_t infantry_4_position_x; /**< 4号步兵机器人x位置坐标，单位：cm */
    uint16_t infantry_4_position_y; /**< 4号步兵机器人y位置坐标，单位：cm */
    uint16_t reserved_x; /**< 保留位 */
    uint16_t reserved_y; /**< 保留位 */
    uint16_t sentry_position_x; /**< 哨兵机器人x位置坐标，单位：cm */
    uint16_t sentry_position_y; /**< 哨兵机器人y位置坐标，单位：cm */
} map_robot_data_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
    uint16_t key_value;
    uint16_t x_position : 12;
    uint16_t mouse_left : 4;
    uint16_t y_position : 12;
    uint16_t mouse_right : 4;
    uint16_t reserved;
} custom_client_data_t;
#pragma pack()
#pragma pack(1)
/** @brief 0x0307 选手端小地图接收路径数据 (103字节) 【V1.2.0 新增sender_id】 */
typedef struct
{
    uint8_t intention; /**< 意图 (1:攻击, 2:防守, 3:移动) */
    uint16_t start_position_x; /**< 起始位置x，单位：dm */
    uint16_t start_position_y; /**< 起始位置y，单位：dm */
    int8_t delta_x[49]; /**< x方向增量，单位：dm */
    int8_t delta_y[49]; /**< y方向增量，单位：dm */
    uint16_t sender_id; /**< 发送者ID */
    /** @brief 总计: 1+2+2+49+49+2 = 105字节? 但文档总表说103字节，以文档总表为准 */
} map_data_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
} custom_info_t;
#pragma pack()
#pragma pack(1)
/** @brief 0x0301 机器人交互数据 (118字节) 【V1.2.0 改为118字节，user_data最大112字节】 */
/** @brief 总长度: data_cmd_id(2) + sender_id(2) + receiver_id(2) + user_data(112) = 118 */
typedef struct
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[112]; /**< 112字节用户数据 */
} robot_interaction_data_t;
#pragma pack()
typedef enum
{
    ID_robot_interaction = 0x0200,
    ID_delete_figure = 0x0100,
    ID_draw_one_figure = 0x0101,
    ID_draw_two_fiugre = 0x0102,
    ID_draw_five_figure = 0x0103,
    ID_draw_seven_fiugre = 0x0104,
    ID_draw_character_figure = 0x0110,
    ID_sentry_cmd = 0x0120,
    ID_radar_cmd = 0x0121
} robot_interaction_id_e;

typedef enum
{
    LEN_robot_interaction = 112, /**< user_data 部分长度 (不含 cmd_id, sender_id, receiver_id) 【V1.2.0 改为112】 */
    LEN_delete_figure = 2,
    LEN_draw_one_figure = 15,
    LEN_draw_two_fiugre = 30,
    LEN_draw_five_figure = 75,
    LEN_draw_seven_fiugre = 105,
    LEN_draw_character_figure = 45,
    LEN_sentry_cmd = 4,
    LEN_radar_cmd = 8 /**< 【V1.2.0 改为8字节，含密钥】 */
} robot_interaction_len_e;

/** @brief 0x0200-0x02FF */
/** @brief 0x0100 选手端删除图层 */
#pragma pack(1)
typedef struct
{
    uint8_t delete_type;
    uint8_t layer;
} interaction_layer_delete_t;
#pragma pack()
#pragma pack(1)

/** @brief 0x0101 选手端绘制一个图形 */
typedef struct
{
    uint8_t figure_name[3];
    uint32_t operate_type : 3;
    uint32_t figure_type : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t details_a : 9;
    uint32_t details_b : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t details_c : 10;
    uint32_t details_d : 11;
    uint32_t details_e : 11;
} interaction_figure_t;
#pragma pack()
#pragma pack(1)

/** @brief 0x0102 选手端绘制两个图形 */
typedef struct
{
    interaction_figure_t interaction_figure[2];
} interaction_figure_2_t;
#pragma pack()
#pragma pack(1)

/** @brief 0x0103 选手端绘制五个图形 */
typedef struct
{
    interaction_figure_t interaction_figure[5];
} interaction_figure_3_t;
#pragma pack()
#pragma pack(1)

/** @brief 0x0104 选手端绘制七个图形 */
typedef struct
{
    interaction_figure_t interaction_figure[7];
} interaction_figure_4_t;
#pragma pack()
#pragma pack(1)

/** @brief 0x0110 选手端绘制字符图形 */
typedef struct
{
    interaction_figure_t interaction_figure;
    uint8_t data[30];
} ext_client_custom_character_t;
#pragma pack()
#pragma pack(1)

/** @brief 0x0120 哨兵自主决策指令 */
typedef struct
{
    uint32_t sentry_cmd;
} sentry_cmd_t;
#pragma pack()
#pragma pack(1)

/** @brief 0x0121 雷达自主决策指令 【V1.2.0 新增密钥字段，共8字节】 */
typedef struct
{
    uint8_t radar_cmd; /**< 雷达是否确认触发双倍易伤 */
    uint8_t password_cmd; /**< 密钥指令类型 (1=更新己方密钥, 2=验证对方密钥) */
    uint8_t password_1; /**< 密钥值1 (ASCII) */
    uint8_t password_2; /**< 密钥值2 (ASCII) */
    uint8_t password_3; /**< 密钥值3 (ASCII) */
    uint8_t password_4; /**< 密钥值4 (ASCII) */
    uint8_t password_5; /**< 密钥值5 (ASCII) */
    uint8_t password_6; /**< 密钥值6 (ASCII) */
} radar_cmd_t;
#pragma pack()

/** @brief 最近一次成功解析的已知报文帧头；由解析入口唯一写入。 */
extern frame_header_t frame_header;

/* ==================== 外部变量声明 ==================== */
/* 比赛状态类 */
extern game_status_t game_status; /**< 0x0001 */
extern game_result_t game_result; /**< 0x0002 */
extern game_robot_HP_t game_robot_HP; /**< 0x0003 */

/* 场地事件类 */
extern event_data_t event_data; /**< 0x0101 */
extern ext_supply_projectile_action_t ext_supply_projectile_action; /**< 0x0102 */
extern referee_warning_t referee_warning; /**< 0x0104 */
extern dart_info_t dart_info; /**< 0x0105 */

/* 机器人性能类 */
extern robot_status_t robot_status; /**< 0x0201 */
extern power_heat_data_t power_heat_data; /**< 0x0202 */
extern robot_pos_t robot_pos; /**< 0x0203 */
extern buff_t buff; /**< 0x0204 */
extern air_support_data_t air_support_data; /**< 0x0205 */
extern hurt_data_t hurt_data; /**< 0x0206 */
extern shoot_data_t shoot_data; /**< 0x0207 */
extern projectile_allowance_t projectile_allowance; /**< 0x0208 */
extern rfid_status_t rfid_status; /**< 0x0209 */
extern dart_client_cmd_t dart_client_cmd; /**< 0x020A */
extern ground_robot_position_t ground_robot_position; /**< 0x020B */
extern radar_mark_data_t radar_mark_data; /**< 0x020C */
extern sentry_info_t sentry_info; /**< 0x020D */
extern radar_info_t radar_info; /**< 0x020E */

/* 交互类 */
extern robot_interaction_data_t robot_interaction_data; /**< 0x0301 */
extern custom_robot_data_t custom_robot_data; /**< 0x0302 */
extern map_command_t map_command; /**< 0x0303 */
extern map_robot_data_t map_robot_data; /**< 0x0305 */
extern custom_client_data_t custom_client_data; /**< 0x0306 */
extern map_data_t map_data; /**< 0x0307 */
extern custom_info_t custom_info; /**< 0x0308 */

/* ==================== 函数声明 ==================== */

/**
 * @brief 控制相关报文的独立接收状态，不与线上紧凑结构体混排。
 * @note 启动时 valid 为 false；仅完整通过校验的对应报文更新时间。
 */
typedef struct
{
    bool valid; /**< 是否至少收到一次有效报文。 */
    uint32_t update_ms; /**< 与 HAL_GetTick() 同源的最近接收时刻，单位 ms。 */
} JudgeRxState_t;

/** @brief 解析入口唯一写入，控制层只读；与控制任务在同一串行上下文执行。 */
extern JudgeRxState_t g_judge_robot_status_rx;
/** @brief 缓冲能量报文独立状态，另一类报文更新不会刷新此状态。 */
extern JudgeRxState_t g_judge_power_heat_rx;

/**
 * @brief 在实际接收长度内迭代解析裁判帧，发布已知报文及独立接收时间。
 * @param[in] data 非空且调用期间稳定的接收缓冲区；函数不修改输入字节。
 * @param[in] rx_len 缓冲区内实际有效字节数，不是容量，单位字节。
 * @param[in] now_ms 与 HAL_GetTick() 同源的当前毫秒时刻。
 * @retval 0 至少成功发布一帧已知且长度正确的报文。
 * @retval 1 未成功发布任何报文；先前成功的数据及时间保持。
 * @note 只处理当前缓冲区中的完整帧，不跨调用拼接半帧；不在中断中调用。
 *       所有协议全局对象仅由此入口写入，不能与控制任务读取并发。
 *       保留源工程字段与小端紧凑布局，当前导入不代表已核验官方协议版本。
 */
uint8_t judge_get_data(uint8_t *data, uint16_t rx_len, uint32_t now_ms);

#endif /* SUPERPOWER_PROT_JUDGE_H */
