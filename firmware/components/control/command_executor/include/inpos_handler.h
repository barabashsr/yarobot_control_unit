/**
 * @file inpos_handler.h
 * @brief INPOS command handler API
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-8: Servo Feedback Processing (Basic InPos)
 *       AC2: INPOS reads all 5 servo axes, returns X:1 Y:0 Z:1 A:1 B:0
 *       AC3: INPOS X reads single axis, returns OK X 1
 *       AC5: INPOS C returns OK C N/A for stepper axes
 *       AC6: Invalid axis returns error
 */

#ifndef INPOS_HANDLER_H
#define INPOS_HANDLER_H

#include "esp_err.h"
#include "command_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle INPOS command
 *
 * Queries servo InPos (in-position) feedback signals.
 *
 * Formats:
 *   INPOS         - Read all 5 servo axes, return "OK X:1 Y:0 Z:1 A:1 B:0" (AC2)
 *   INPOS X       - Read single axis, return "OK X 1" (AC3)
 *   INPOS C       - Stepper axis, return "OK C N/A" (AC5)
 *   INPOS Q       - Invalid axis, return "ERROR E002 Invalid axis" (AC6)
 *
 * @param[in] cmd Parsed command structure
 * @param[out] response Buffer for response string
 * @param[in] resp_len Size of response buffer
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t handle_inpos(const ParsedCommand* cmd, char* response, size_t resp_len);

/**
 * @brief Register INPOS command handler
 *
 * Registers the INPOS handler with the command executor.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t inpos_handler_register(void);

#ifdef __cplusplus
}
#endif

#endif // INPOS_HANDLER_H
