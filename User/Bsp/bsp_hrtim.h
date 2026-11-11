/**
 * @file bsp_hrtim.h
 * @brief 广义占空比到本板双半桥 PWM 的转换及功率输出启停接口。
 * @details 广义占空比为电容侧电压与底盘侧电压之比，无单位。
 *          Timer A 接电容侧，Timer B 接底盘侧；CH1 接下管，CH2 接上管。
 * @note 由 PowerCtrlTask 接入启动；ADC1 DMA 上下文中闭环由 PowerCtrl 管理输出，
 *       开环由回调单次开启固定 PWM；须满足下述 CubeMX 前提。
 */

#ifndef SUPERPOWER_BSP_HRTIM_H
#define SUPERPOWER_BSP_HRTIM_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ==================== 调制参数 ==================== */

/**
 * @brief 广义占空比限幅下限，无单位。
 * @note 来源：港科 RM2024 配套 PowerManager.cpp 的 updatePWM()，本板范围待验证。
 */
#define BSP_HRTIM_GENERALIZED_DUTY_MIN (0.05f)

/** @brief 同源广义占空比限幅上限，无单位；不代表硬件额定范围。 */
#define BSP_HRTIM_GENERALIZED_DUTY_MAX (10.0f)

/** @brief 同源环路占空比限幅上限，无单位；不代表硬件额定范围。 */
#define BSP_HRTIM_LOOP_DUTY_MAX (0.2f)

/** @brief 同源非中间模式的上管基准占空比，无单位；保留周期性自举刷新。 */
#define BSP_HRTIM_BASE_UPPER_DUTY (0.9f)

/** @brief 同源中间模式占空比分配系数，无单位，本板脉宽及死区裕量待验证。 */
#define BSP_HRTIM_BUCK_BOOST_SCALE (0.4f)

/** @brief 同源进入中间模式的下界，严格大于时才可能进入，无单位。 */
#define BSP_HRTIM_BUCK_BOOST_ENTER_MIN (0.9f)

/** @brief 同源进入中间模式的上界，严格小于时才可能进入，无单位。 */
#define BSP_HRTIM_BUCK_BOOST_ENTER_MAX (1.111f)

/** @brief 同源退出中间模式的下界，严格小于时退出，无单位。 */
#define BSP_HRTIM_BUCK_BOOST_EXIT_MIN (0.8f)

/** @brief 同源退出中间模式的上界，严格大于时退出，无单位。 */
#define BSP_HRTIM_BUCK_BOOST_EXIT_MAX (1.25f)

/* ==================== 公共接口 ==================== */
/**
 * @brief 将广义占空比转换为两侧 PWM 比较值，并写入 A/B 的预装载寄存器。
 * @param[in] generalized_duty 理想电容侧电压/底盘侧电压，无单位，由本接口统一限幅。
 * @pre 已调用 BspHrtim_Init()；输入统一限幅至 MIN/MAX，不将越界视为故障。
 * @pre CubeMX 已为 A/B 开启 PreloadEnable、ResetUpdate，使用独立 UpdateGating；
 *      关闭其他更新来源。主/A/B 为相同且固定的周期、向上计数，A/B 保持半周期错相。
 *      CH1 使用 CMP1 置位、CMP2 复位，CH2 由正死区互补生成，四路均为高有效。
 * @pre 调用方唯一持有 A/B 比较值及其更新禁止位；调用不可重入，也不与其他 HRTIM
 *      公共寄存器或 HAL 句柄写入并发。运行期两次调用至少间隔一个完整 PWM 周期。
 * @note 只更新 CMP1/CMP2，保留 CMP3 的 ADC 触发。先禁止 A/B 装载、完整写入四值，
 *       再恢复原更新禁止位；两侧各自在下次复位/回卷时生效，相差半周期。
 *       不强制软件更新、不自动启用输出、不补偿死区和驱动延迟，不保证实测电压比。
 *       零、负数及无穷按上下限限幅；NaN 不修改迟滞状态或寄存器，不改变输出启停。
 */
void BspHrtim_SetGeneralizedDuty(float generalized_duty);

/**
 * @brief 统一开启 TA1、TA2、TB1、TB2 四路功率输出。
 * @pre 已初始化；SetGeneralizedDuty() 所述固定外设配置成立，计数器正常运行且更新已使能。
 *      最近一次比较值设置后已运行至少一个完整 PWM 周期，确保两侧比较值均已装载。
 * @pre 闭环调用方已确认完整 ADC 校准、反馈及启动条件成立，并已设置有效平衡占空比。
 *      开环调用方使用 CubeMX 配置的固定 50% 窗口，并确认外部已断开 MOS 驱动供电或信号；
 *      开环的固定比较值不能作为闭环功率启动值；开环不执行软件错误或远程保护判断。
 *      闭环调用方须确认 PWM 就绪且无远程保护请求。
 *      与本模块其他调用及 HRTIM HAL 句柄写入互斥。
 * @note 不读取校准状态、不启动计数器、不设置默认占空比；HAL 失败进入 Error_Handler()。
 *       此接口会开启四路引脚输出，软件无法同时保证所连 MOS 不动作。
 */
void BspHrtim_EnableOutput(void);

/**
 * @brief 统一关闭 TA1、TA2、TB1、TB2 四路功率输出，保留采样触发计数器运行。
 * @pre MX_HRTIM1_Init() 已完成；与本模块其他调用及 HRTIM HAL 句柄写入互斥。
 * @note 不清除迟滞状态或比较值，不操作计数器；HAL 失败进入 Error_Handler()。
 *       不是可在任意中断中抢占执行的硬件故障保护接口。
 */
void BspHrtim_DisableOutput(void);

#ifdef __cplusplus
}
#endif

#endif /* SUPERPOWER_BSP_HRTIM_H */
