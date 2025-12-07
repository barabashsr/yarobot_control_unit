/**
 * @file position_handler.h
 * @brief POS command handler for position query
 * @author YaRobot Team
 * @date 2025
 *
 * @note Handles CMD_POS command: POS [axis]
 *       Queries current position of one or all axes.
 *
 * @note Story 3-10 implementation - AC5-AC7
 */

#ifndef POSITION_HANDLER_H
#define POSITION_HANDLER_H

#include "esp_err.h"
#include "command_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle POS command
 *
 * Parses and executes position query command.
 * Format: POS [axis]
 *
 * - POS: Return all 8 axis positions (AC5)
 * - POS X: Return single axis position (AC6)
 *
 * @param[in] cmd Parsed command with optional axis
 * @param[out] response Buffer for response string
 * @param[in] resp_len Size of response buffer
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG for invalid axis
 *
 * Response format:
 * - All axes: "OK X:0.001000 Y:0.000000 Z:...\r\n" (AC5)
 * - Single axis: "OK 0.001000\r\n" (AC6)
 * - Invalid axis: "ERROR E002 Invalid axis\r\n" (AC7)
 */
esp_err_t handle_position(const ParsedCommand* cmd, char* response, size_t resp_len);

/**
 * @brief Register POS command handler
 *
 * Registers the POS command with the command executor.
 * Should be called during system initialization.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if command executor not initialized
 */
esp_err_t position_handler_register(void);

#ifdef __cplusplus
}
#endif

#endif // POSITION_HANDLER_H
