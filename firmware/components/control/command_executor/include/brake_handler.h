/**
 * @file brake_handler.h
 * @brief BRAKE command handler API
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-5: Brake Control System
 */

#ifndef BRAKE_HANDLER_H
#define BRAKE_HANDLER_H

#include "esp_err.h"
#include "command_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle BRAKE command
 *
 * Syntax: BRAKE <axis> <0|1>
 * - 0 = engage brake
 * - 1 = release brake
 *
 * @param[in] cmd Parsed command structure
 * @param[out] response Response buffer
 * @param[in] resp_len Size of response buffer
 *
 * @return ESP_OK on success, error code otherwise
 *
 * @note Only works for axes with BRAKE_MANUAL strategy
 * @note Returns ERR_BRAKE_AUTO for auto-managed axes
 * @note Returns ERR_AXIS_NO_BRAKE for axes without brake hardware (C, D, E)
 */
esp_err_t handle_brake(const ParsedCommand* cmd, char* response, size_t resp_len);

/**
 * @brief Register BRAKE command handler
 *
 * @return ESP_OK on success
 */
esp_err_t brake_handler_register(void);

#ifdef __cplusplus
}
#endif

#endif // BRAKE_HANDLER_H
