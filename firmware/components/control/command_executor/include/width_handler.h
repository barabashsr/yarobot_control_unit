/**
 * @file width_handler.h
 * @brief WIDTH command handler header
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-9: C-Axis Floating Switch (Object Width Measurement)
 *       AC4: WIDTH C command returns last measurement or -1.000 if none
 *       AC7: WIDTH X (non-C axis) returns error ERR_WIDTH_C_ONLY
 */

#ifndef WIDTH_HANDLER_H
#define WIDTH_HANDLER_H

#include "esp_err.h"
#include "command_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle WIDTH command
 *
 * Returns the last width measurement from C axis floating switch.
 *
 * Command format: WIDTH [axis]
 * - WIDTH C: Returns "OK C <value>" with last measurement
 * - WIDTH C (no measurement): Returns "OK C -1.000"
 * - WIDTH X (non-C): Returns "ERROR E040 WIDTH only for C axis"
 *
 * @param[in] cmd Parsed command structure
 * @param[out] response Buffer for response string
 * @param[in] resp_len Size of response buffer
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if cmd or response is NULL
 */
esp_err_t handle_width(const ParsedCommand* cmd, char* response, size_t resp_len);

/**
 * @brief Register WIDTH command handler
 *
 * Registers the CMD_WIDTH handler with the command executor.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if command table full
 */
esp_err_t width_handler_register(void);

#ifdef __cplusplus
}
#endif

#endif // WIDTH_HANDLER_H
