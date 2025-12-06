/**
 * @file move_handler.h
 * @brief MOVE command handler for absolute positioning
 * @author YaRobot Team
 * @date 2025
 *
 * @note Handles CMD_MOVE command: MOVE <axis> <position> [velocity]
 *       Moves specified axis to absolute position.
 */

#ifndef MOVE_HANDLER_H
#define MOVE_HANDLER_H

#include "esp_err.h"
#include "command_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle MOVE command
 *
 * Parses and executes absolute move command.
 * Format: MOVE <axis> <position> [velocity]
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
esp_err_t handle_move(const ParsedCommand* cmd, char* response, size_t resp_len);

/**
 * @brief Register MOVE command handler
 *
 * Registers the MOVE command with the command executor.
 * Should be called during system initialization.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if command executor not initialized
 */
esp_err_t move_handler_register(void);

#ifdef __cplusplus
}
#endif

#endif // MOVE_HANDLER_H
