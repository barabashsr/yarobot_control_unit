/**
 * @file dout_handler.h
 * @brief DOUT command handler API
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-7: General-Purpose Digital I/O
 *       AC4: DOUT pin 1 sets output HIGH
 *       AC5: DOUT pin 0 sets output LOW
 *       AC6: DOUT query returns all outputs as binary
 *       AC7: DOUT alias sets output by alias name
 *       AC10: Invalid pin/alias returns error
 */

#ifndef DOUT_HANDLER_H
#define DOUT_HANDLER_H

#include "esp_err.h"
#include "command_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle DOUT command
 *
 * Parses DOUT command and controls digital outputs.
 *
 * Formats:
 *   DOUT           - Query all 8 outputs, return "OK 0b00100000" (AC6)
 *   DOUT 5 1       - Set output 5 HIGH, return "OK" (AC4)
 *   DOUT 5 0       - Set output 5 LOW, return "OK" (AC5)
 *   DOUT LIGHT 1   - Set output by alias, return "OK" (AC7)
 *
 * @param[in] cmd Parsed command structure
 * @param[out] response Buffer for response string
 * @param[in] resp_len Size of response buffer
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t handle_dout(const ParsedCommand* cmd, char* response, size_t resp_len);

/**
 * @brief Register DOUT command handler
 *
 * Registers the DOUT handler with the command executor.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t dout_handler_register(void);

#ifdef __cplusplus
}
#endif

#endif // DOUT_HANDLER_H
