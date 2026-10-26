/**
 * @file module_template.h
 * @brief 独立用户模块的接口与中文注释模板。
 * @details
 *   本模板只演示实例、状态、错误码及初始化接口，不访问硬件，不参与固件构建。
 *   新模块应同步替换文件名、保护宏、类型、函数和枚举前缀。
 * @note 调用上下文为初始化阶段；同一实例不允许并发修改。
 */

#ifndef SUPERPOWER_MODULE_TEMPLATE_H
#define SUPERPOWER_MODULE_TEMPLATE_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ==================== 数据类型定义 ==================== */

/** @brief 模块运行状态。 */
typedef enum
{
    MODULE_STATE_UNINITIALIZED = 0, /**< 尚未初始化。 */
    MODULE_STATE_READY              /**< 已完成初始化。 */
} ModuleState_e;

/** @brief 模块操作结果。 */
typedef enum
{
    MODULE_RESULT_OK = 0,        /**< 操作成功。 */
    MODULE_RESULT_INVALID_PARAM /**< 必需参数为空。 */
} ModuleResult_e;

/**
 * @brief 调用方持有的模块实例。
 * @note 生命周期由调用方管理；外部通过接口更新状态，不直接写成员。
 */
typedef struct
{
    ModuleState_e state; /**< 模块状态，无物理单位；有效值见 ModuleState_e。 */
} Module_t;

/* ==================== 公共接口声明 ==================== */

/**
 * @brief 将有效实例初始化或重置到就绪状态。
 * @param[out] module 待初始化实例，必须指向有效可写的 Module_t；不要求预先初始化。
 * @retval MODULE_RESULT_OK 已将实例状态设为 MODULE_STATE_READY。
 * @retval MODULE_RESULT_INVALID_PARAM module 为 NULL；不写入任何实例。
 * @note 可重复调用；不分配内存、不阻塞、不访问外设。
 *       调用方保证整个调用期间对该实例具有独占写入权。
 */
ModuleResult_e Module_Init(Module_t *module);

#ifdef __cplusplus
}
#endif

#endif /* SUPERPOWER_MODULE_TEMPLATE_H */
