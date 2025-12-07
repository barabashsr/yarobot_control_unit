/**
 * @file stop_handler.h
 * @brief STOP command handler for controlled motion stop
 * @author YaRobot Team
 * @date 2025
 *
 * @note Handles CMD_STOP command: STOP [axis]
 *       Stops motion with controlled deceleration.
 *
 * @note Story 3-10 implementation - AC13-AC16
 */

#ifndef STOP_HANDLER_H
#define STOP_HANDLER_H

#include "esp_err.h"
#include "command_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle STOP command
 *
 * Parses and executes stop command.
 * Format: STOP [axis]
 *
 * - STOP X: Stop X-axis with controlled deceleration (AC13)
 * - STOP: Stop all moving axes (AC14)
 * - STOP while idle returns OK (AC15)
 *
 * @param[in] cmd Parsed command with optional axis
 * @param[out] response Buffer for response string
 * @param[in] resp_len Size of response buffer
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG for invalid axis
 *
 * Response format:
 * - Success: "OK\r\n" (even if already stopped - AC15)
 * - Invalid axis: "ERROR E002 Invalid axis\r\n" (AC16)
 */
esp_err_t handle_stop(const ParsedCommand* cmd, char* response, size_t resp_len);

/**
 * @brief Register STOP command handler
 *
 * Registers the STOP command with the command executor.
 * Should be called during system initialization.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if command executor not initialized
 */
esp_err_t stop_handler_register(void);

#ifdef __cplusplus
}
#endif

#endif // STOP_HANDLER_H
