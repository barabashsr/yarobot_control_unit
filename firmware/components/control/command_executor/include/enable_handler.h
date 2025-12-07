/**
 * @file enable_handler.h
 * @brief EN command handler for motor enable/disable control
 * @author YaRobot Team
 * @date 2025
 *
 * @note Handles CMD_EN command: EN <axis> <0|1>
 *       Enables or disables the specified motor axis.
 *
 * @note Story 3-10 implementation - AC1-AC4
 */

#ifndef ENABLE_HANDLER_H
#define ENABLE_HANDLER_H

#include "esp_err.h"
#include "command_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle EN command
 *
 * Parses and executes enable/disable command.
 * Format: EN <axis> <0|1>
 *
 * - EN X 1: Enable X-axis, state DISABLED -> IDLE (AC1)
 * - EN X 0: Disable X-axis, immediate stop if moving, state -> DISABLED (AC2, AC3)
 *
 * @param[in] cmd Parsed command with axis and enable state parameter
 * @param[out] response Buffer for response string
 * @param[in] resp_len Size of response buffer
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG for invalid axis or parameter
 *
 * Response format:
 * - Success: "OK\r\n"
 * - Invalid axis: "ERROR E002 Invalid axis\r\n" (AC4)
 * - Invalid parameter: "ERROR E003 Invalid parameter\r\n"
 */
esp_err_t handle_enable(const ParsedCommand* cmd, char* response, size_t resp_len);

/**
 * @brief Register EN command handler
 *
 * Registers the EN command with the command executor.
 * Should be called during system initialization.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if command executor not initialized
 */
esp_err_t enable_handler_register(void);

#ifdef __cplusplus
}
#endif

#endif // ENABLE_HANDLER_H
