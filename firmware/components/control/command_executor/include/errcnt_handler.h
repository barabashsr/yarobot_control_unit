/**
 * @file errcnt_handler.h
 * @brief ERRCNT command handler - Error count query
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-10: Error Tracking & Recovery
 *       AC1: ERRCNT returns global error counts per category
 *       AC2: ERRCNT X returns per-axis error counts
 *       AC7: ERRCNT CLR resets all error counts
 */

#ifndef ERRCNT_HANDLER_H
#define ERRCNT_HANDLER_H

#include "esp_err.h"
#include "command_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle ERRCNT command
 *
 * Queries or clears error counts.
 *
 * Usage:
 *   ERRCNT          - Returns global error counts per category
 *   ERRCNT X        - Returns per-axis error counts for axis X
 *   ERRCNT CLR      - Clears all error counts
 *
 * Response:
 *   OK I2C:n TIMEOUT:n LIMIT:n ESTOP:n POSLOS:n  - Global counts
 *   OK X LIMIT:n POSLOS:n TIMEOUT:n              - Per-axis counts
 *   OK                                            - Counts cleared
 *   ERROR E002                                    - Invalid axis
 *
 * @param[in] cmd Parsed command structure
 * @param[out] response Response buffer
 * @param[in] resp_len Size of response buffer
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t handle_errcnt(const ParsedCommand* cmd, char* response, size_t resp_len);

/**
 * @brief Register ERRCNT command handler
 *
 * Registers the CMD_ERRCNT handler with the command executor.
 *
 * @return esp_err_t
 *         - ESP_OK: Handler registered successfully
 *         - ESP_ERR_NO_MEM: Command table full
 */
esp_err_t errcnt_handler_register(void);

#ifdef __cplusplus
}
#endif

#endif // ERRCNT_HANDLER_H
