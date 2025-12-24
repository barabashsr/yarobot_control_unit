/**
 * @file clrerr_handler.h
 * @brief CLRERR command handler - Clear error state
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-10: Error Tracking & Recovery
 *       AC4: CLRERR X clears error state for single axis
 *       AC4: CLRERR clears error state for all axes
 */

#ifndef CLRERR_HANDLER_H
#define CLRERR_HANDLER_H

#include "esp_err.h"
#include "command_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle CLRERR command
 *
 * Clears axis error state to allow motion commands.
 *
 * Usage:
 *   CLRERR          - Clears error state for all axes
 *   CLRERR X        - Clears error state for axis X
 *
 * Response:
 *   OK              - Error state cleared
 *   ERROR E002      - Invalid axis
 *
 * @param[in] cmd Parsed command structure
 * @param[out] response Response buffer
 * @param[in] resp_len Size of response buffer
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t handle_clrerr(const ParsedCommand* cmd, char* response, size_t resp_len);

/**
 * @brief Register CLRERR command handler
 *
 * Registers the CMD_CLRERR handler with the command executor.
 *
 * @return esp_err_t
 *         - ESP_OK: Handler registered successfully
 *         - ESP_ERR_NO_MEM: Command table full
 */
esp_err_t clrerr_handler_register(void);

#ifdef __cplusplus
}
#endif

#endif // CLRERR_HANDLER_H
