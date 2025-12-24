/**
 * @file i2c_handler.h
 * @brief I2C command handler interface
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-11: I2C Communication Health
 *       AC5: CMD_I2C returns health status for all tracked devices
 *       AC6: CMD_I2C SCAN returns found device addresses
 */

#ifndef I2C_HANDLER_H
#define I2C_HANDLER_H

#include "esp_err.h"
#include "command_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle I2C command
 *
 * Supports:
 * - I2C: Returns health status for all tracked devices
 *   Format: "OK I2C 0x20:ONLINE:100:0 0x21:OFFLINE:50:5"
 *
 * - I2C SCAN: Scans I2C bus and returns found addresses
 *   Format: "OK I2C SCAN 0x20 0x21"
 *
 * @param[in] cmd Parsed command structure
 * @param[out] response Response buffer
 * @param[in] resp_len Response buffer length
 *
 * @return esp_err_t
 *         - ESP_OK: Command executed, response written
 *         - ESP_ERR_INVALID_ARG: Invalid parameters
 */
esp_err_t handle_i2c(const ParsedCommand* cmd, char* response, size_t resp_len);

/**
 * @brief Register I2C command handler with command executor
 *
 * @return esp_err_t
 *         - ESP_OK: Handler registered
 *         - ESP_ERR_NO_MEM: Registration table full
 */
esp_err_t i2c_handler_register(void);

#ifdef __cplusplus
}
#endif

#endif // I2C_HANDLER_H
