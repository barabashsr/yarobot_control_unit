/**
 * @file movr_handler.h
 * @brief MOVR command handler for relative positioning
 * @author YaRobot Team
 * @date 2025
 *
 * @note Handles CMD_MOVR command: MOVR <axis> <delta> [velocity]
 *       Moves specified axis relative to current position.
 */

#ifndef MOVR_HANDLER_H
#define MOVR_HANDLER_H

#include "esp_err.h"
#include "command_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle MOVR command
 *
 * Parses and executes relative move command.
 * Format: MOVR <axis> <delta> [velocity]
 *
 * @param[in] cmd Parsed command with axis and parameters
 * @param[out] response Buffer for response string
 * @param[in] resp_len Size of response buffer
 *
 * @return ESP_OK on success (move started)
 * @return ESP_ERR_INVALID_ARG for invalid axis or position limit
 * @return ESP_ERR_INVALID_STATE for disabled axis
 *
 * Response format:
 * - Success: "OK\r\n"
 * - Invalid axis: "ERROR E002 Invalid axis\r\n"
 * - Axis disabled: "ERROR E004 Axis not enabled\r\n"
 * - Position limit: "ERROR E005 Position limit exceeded\r\n"
 */
esp_err_t handle_movr(const ParsedCommand* cmd, char* response, size_t resp_len);

/**
 * @brief Register MOVR command handler
 *
 * Registers the MOVR command with the command executor.
 * Should be called during system initialization.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if command executor not initialized
 */
esp_err_t movr_handler_register(void);

#ifdef __cplusplus
}
#endif

#endif // MOVR_HANDLER_H
