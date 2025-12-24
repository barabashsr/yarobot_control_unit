/**
 * @file din_handler.h
 * @brief DIN command handler API
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-7: General-Purpose Digital I/O
 *       AC1: DIN reads all 4 inputs, returns binary format
 *       AC2: DIN pin reads single input
 *       AC3: DIN alias reads by alias name
 *       AC10: Invalid pin/alias returns error
 */

#ifndef DIN_HANDLER_H
#define DIN_HANDLER_H

#include "esp_err.h"
#include "command_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle DIN command
 *
 * Parses DIN command and reads digital inputs.
 *
 * Formats:
 *   DIN           - Read all 4 inputs, return "OK 0b1010" (AC1)
 *   DIN 2         - Read single input, return "OK DIN2 1" (AC2)
 *   DIN SENSOR1   - Read by alias, return "OK SENSOR1 0" (AC3)
 *
 * @param[in] cmd Parsed command structure
 * @param[out] response Buffer for response string
 * @param[in] resp_len Size of response buffer
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t handle_din(const ParsedCommand* cmd, char* response, size_t resp_len);

/**
 * @brief Register DIN command handler
 *
 * Registers the DIN handler with the command executor.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t din_handler_register(void);

#ifdef __cplusplus
}
#endif

#endif // DIN_HANDLER_H
