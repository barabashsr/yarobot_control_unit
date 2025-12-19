/**
 * @file lim_handler.c
 * @brief LIM command handler implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-2: Limit Switch Monitoring
 *       AC1: LIM returns all axes: OK X:00 Y:00 Z:01 A:00 B:00 C:10 D:00 E:--
 *       AC2: LIM X returns single axis: OK X MIN:0 MAX:0
 */

#include "safety_monitor.h"
#include "command_executor.h"
#include "command_parser.h"
#include "response_formatter.h"
#include "config_commands.h"
#include "config_limits.h"

#include "esp_log.h"

#include <stdio.h>
#include <string.h>

static const char* TAG = "LIM_HDL";

/** @brief Axis names for formatting */
static const char AXIS_NAMES[] = "XYZABCDE";

/**
 * @brief Handle LIM command
 *
 * @param cmd Parsed command
 * @param response Response buffer
 * @param resp_len Response buffer length
 * @return esp_err_t
 */
static esp_err_t handle_lim(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    if (cmd == NULL || response == NULL || resp_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if safety monitor is initialized
    if (!safety_monitor_is_initialized()) {
        ESP_LOGW(TAG, "Safety monitor not initialized, reading anyway");
        // We can still try to read - just won't have debounce etc.
    }

    // Check if single axis query
    if (cmd->axis != '\0') {
        // Single axis query (AC2)
        int8_t axis_idx = axis_to_index(cmd->axis);
        if (axis_idx < 0 || axis_idx >= LIMIT_NUM_AXES) {
            ESP_LOGD(TAG, "Invalid axis: %c", cmd->axis);
            return format_error(response, resp_len, ERR_INVALID_AXIS, MSG_INVALID_AXIS);
        }

        bool min_active, max_active;
        esp_err_t err = safety_monitor_get_axis_limits((uint8_t)axis_idx, &min_active, &max_active);
        if (err != ESP_OK) {
            return format_error(response, resp_len, ERR_COMMUNICATION, MSG_COMMUNICATION);
        }

        // E axis has no limits
        if (axis_idx == SAFETY_AXIS_E) {
            return format_ok_data(response, resp_len, "%c --", cmd->axis);
        }

        // Format: OK X MIN:0 MAX:0
        return format_ok_data(response, resp_len, "%c MIN:%d MAX:%d",
                             cmd->axis, min_active ? 1 : 0, max_active ? 1 : 0);
    }

    // All axes query (AC1)
    // Format: OK X:00 Y:00 Z:01 A:00 B:00 C:10 D:00 E:--

    // Build response string
    char data[128];
    int pos = 0;

    for (uint8_t axis = 0; axis < LIMIT_NUM_AXES; axis++) {
        if (axis > 0 && pos < (int)(sizeof(data) - 1)) {
            data[pos++] = ' ';
        }

        bool min_active, max_active;
        esp_err_t err = safety_monitor_get_axis_limits(axis, &min_active, &max_active);
        if (err != ESP_OK) {
            min_active = false;
            max_active = false;
        }

        // E axis has no physical limits
        if (axis == SAFETY_AXIS_E) {
            int written = snprintf(&data[pos], sizeof(data) - pos, "%c:--", AXIS_NAMES[axis]);
            if (written > 0) {
                pos += written;
            }
        } else {
            // Format as 2-digit binary: bit0=min, bit1=max
            // 00 = both off, 01 = min on, 10 = max on, 11 = both on
            uint8_t state = (min_active ? 1 : 0) | (max_active ? 2 : 0);
            int written = snprintf(&data[pos], sizeof(data) - pos, "%c:%d%d",
                                   AXIS_NAMES[axis],
                                   (state >> 1) & 1,  // max bit first (MSB)
                                   state & 1);         // min bit second (LSB)
            if (written > 0) {
                pos += written;
            }
        }
    }

    data[pos] = '\0';
    return format_ok_data(response, resp_len, "%s", data);
}

/**
 * @brief Handle LIMT (LIM Test) command - inject test state
 *
 * LIMT <hex_value> - Inject 16-bit test state (e.g., LIMT 0x0005 sets X MIN and MAX active)
 * LIMT RESET       - Restore hardware state
 *
 * Bit mapping: bit0=X_MIN, bit1=X_MAX, bit2=Y_MIN, bit3=Y_MAX, etc.
 */
static esp_err_t handle_limt(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    if (cmd == NULL || response == NULL || resp_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // LIMT requires parameter
    if (cmd->param_count < 1) {
        // Show usage
        return format_ok_data(response, resp_len,
            "Usage: LIMT <hex> or LIMT 0 (reset). Bits: 0=XMIN,1=XMAX,2=YMIN...");
    }

    uint16_t test_state = (uint16_t)cmd->params[0];

    if (test_state == 0) {
        // Reset to hardware state
        esp_err_t err = safety_monitor_update_cache();
        if (err == ESP_OK) {
            return format_ok_data(response, resp_len, "Reset to hardware state");
        }
        return format_error(response, resp_len, ERR_COMMUNICATION, MSG_COMMUNICATION);
    }

    // Inject test state
    safety_monitor_inject_test_state(test_state);
    return format_ok_data(response, resp_len, "Injected 0x%04X", test_state);
}

esp_err_t lim_handler_register(void)
{
    esp_err_t ret;

    // Register LIM command
    CommandEntry lim_entry = {
        .verb = CMD_LIM,
        .handler = handle_lim,
        .allowed_states = STATE_ANY
    };
    ret = cmd_executor_register(&lim_entry);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register LIM handler: 0x%x", ret);
        return ret;
    }

    // Register LIMT (test) command
    CommandEntry limt_entry = {
        .verb = "LIMT",
        .handler = handle_limt,
        .allowed_states = STATE_ANY
    };
    ret = cmd_executor_register(&limt_entry);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register LIMT handler: 0x%x", ret);
        // Non-fatal - test command is optional
    }

    ESP_LOGI(TAG, "Registered LIM and LIMT handlers");
    return ESP_OK;
}
