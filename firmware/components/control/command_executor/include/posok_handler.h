/**
 * @file posok_handler.h
 * @brief POSOK command handler - Position loss acknowledgment
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-6: Position Loss Detection
 *       AC4: POSOK X clears POSLOS for single axis
 *       AC5: POSOK clears POSLOS for all axes
 */

#ifndef POSOK_HANDLER_H
#define POSOK_HANDLER_H

#include "esp_err.h"
#include "command_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle POSOK command
 *
 * Acknowledges position loss for one or all axes.
 *
 * Usage:
 *   POSOK X      - Clear POSLOS for axis X, return to IDLE
 *   POSOK        - Clear POSLOS for all servo axes
 *
 * Response:
 *   OK           - POSLOS cleared successfully
 *   ERROR E037   - Axis not in POSLOS state (single axis mode)
 *   ERROR E002   - Invalid axis (not a servo axis)
 *
 * @param[in] cmd Parsed command structure
 * @param[out] response Response buffer
 * @param[in] resp_len Size of response buffer
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t handle_posok(const ParsedCommand* cmd, char* response, size_t resp_len);

/**
 * @brief Register POSOK command handler
 *
 * Registers the CMD_POSOK handler with the command executor.
 *
 * @return esp_err_t
 *         - ESP_OK: Handler registered successfully
 *         - ESP_ERR_NO_MEM: Command table full
 */
esp_err_t posok_handler_register(void);

#ifdef __cplusplus
}
#endif

#endif // POSOK_HANDLER_H
