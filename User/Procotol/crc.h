/**
 * @file crc.h
 * @brief 裁判协议 CRC8、CRC16 与 CRC-CCITT 公共接口。
 * @note 公共名称和参数沿用 Taurus；无 HAL 或 RTOS 依赖。
 */
#ifndef SUPERPOWER_CRC_H
#define SUPERPOWER_CRC_H

#include <stdint.h>

/**
 * @brief 计算 CRC8，读取 length 字节，不写入输入。
 * @param[in] pdata 非空且至少含 length 字节的稳定缓冲区。
 * @param[in] length 参与计算的字节数。
 * @param[in] crc8 调用方指定的 CRC 初值。
 * @return 计算得到的 CRC8。
 */
uint8_t crc8_get_checksum(uint8_t *pdata, uint16_t length, uint8_t crc8);

/**
 * @brief 以 0xff 初值校验最后一字节的 CRC8。
 * @param[in] pdata 含数据及末尾 CRC8 的稳定缓冲区。
 * @param[in] length 缓冲区总字节数，包含 CRC8。
 * @retval 1 校验正确。
 * @retval 0 校验错误、空指针或 length 不大于 2。
 */
uint32_t crc8_verify_checksum(uint8_t *pdata, uint16_t length);

/**
 * @brief 以 0xff 初值计算并写入末尾一字节 CRC8。
 * @param[in,out] pdata 可写缓冲区，至少含 length 字节。
 * @param[in] length 缓冲区总字节数，包含 CRC8。
 * @note 空指针或 length 不大于 2 时不写入。
 */
void crc8_set_checksum(uint8_t *pdata, uint16_t length);

/**
 * @brief 计算 CRC16，读取 length 字节，不写入输入。
 * @param[in] pdata 至少含 length 字节的稳定缓冲区。
 * @param[in] length 参与计算的字节数。
 * @param[in] crc16 调用方指定的 CRC 初值。
 * @return CRC16；空指针时沿用源接口返回 0xffff。
 */
uint16_t crc16_get_checksum(uint8_t *pdata, uint32_t length, uint16_t crc16);

/**
 * @brief 以 0xffff 初值校验末尾两字节的小端 CRC16。
 * @param[in] pdata 含数据及 CRC16 的稳定缓冲区。
 * @param[in] length 缓冲区总字节数，包含 CRC16。
 * @retval 1 校验正确。
 * @retval 0 校验错误、空指针或 length 不大于 2。
 */
uint32_t crc16_verify_checksum(uint8_t *pdata, uint16_t length);

/**
 * @brief 以 0xffff 初值计算并写入末尾两字节小端 CRC16。
 * @param[in,out] pdata 可写缓冲区，至少含 length 字节。
 * @param[in] length 缓冲区总字节数，包含 CRC16。
 * @note 空指针或 length 不大于 2 时不写入。
 */
void crc16_set_checksum(uint8_t *pdata, uint16_t length);

/**
 * @brief 计算源工程表驱动 CRC-CCITT。
 * @param[in] pdata 非空且至少含 length 字节的稳定缓冲区。
 * @param[in] length 参与计算的字节数。
 * @param[in] crc_init 调用方指定的初值。
 * @return 计算得到的 CRC-CCITT。
 */
uint16_t crc_ccitt_get_checksum(uint8_t const *pdata, uint16_t length, uint16_t crc_init);

#endif /* SUPERPOWER_CRC_H */
