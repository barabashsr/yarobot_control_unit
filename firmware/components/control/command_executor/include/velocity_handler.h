/**
 * @file velocity_handler.h
 * @brief VEL command handler for velocity (jog) mode
 * @author YaRobot Team
 * @date 2025
 *
 * @note Handles CMD_VEL command: VEL <axis> <velocity>
 *       Starts continuous velocity mode motion.
 *
 * @note Story 3-10 implementation - AC8-AC12
 */

#ifndef VELOCITY_HANDLER_H
#define VELOCITY_HANDLER_H

#include "esp_err.h"
#include "command_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle VEL command
 *
 * Parses and executes velocity mode command.
 * Format: VEL <axis> <velocity>
 *
 * - VEL X 0.050: Move X at 0.050 units/s continuously (AC8)
 * - VEL X -0.025: Move X in negative direction (AC9)
 * - Velocity exceeding max is clamped (AC11)
 * - New VEL during motion blends to new velocity (AC12)
 *
 * @param[in] cmd Parsed command with axis and velocity parameter
 * @param[out] response Buffer for response string
 * @param[in] resp_len Size of response buffer
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG for invalid axis or parameter
 * @return ESP_ERR_INVALID_STATE for disabled axis
 *
 * Response format:
 * - Success: "OK\r\n"
 * - Invalid axis: "ERROR E002 Invalid axis\r\n"
 * - Axis not enabled: "ERROR E004 Axis not enabled\r\n" (AC10)
 */
esp_err_t handle_velocity(const ParsedCommand* cmd, char* response, size_t resp_len);

/**
 * @brief Register VEL command handler
 *
 * Registers the VEL command with the command executor.
 * Should be called during system initialization.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if command executor not initialized
 */
esp_err_t velocity_handler_register(void);

#ifdef __cplusplus
}
#endif

#endif // VELOCITY_HANDLER_H
