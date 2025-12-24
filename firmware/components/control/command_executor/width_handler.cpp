/**
 * @file width_handler.cpp
 * @brief WIDTH command handler implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-9: C-Axis Floating Switch (Object Width Measurement)
 *       AC4: WIDTH C returns last measurement or -1.000 if none
 *       AC7: Invalid axis (non-C) returns error ERR_WIDTH_C_ONLY
 */

#include "width_handler.h"
#include "floating_switch.h"
#include "command_executor.h"
#include "response_formatter.h"
#include "config_commands.h"
#include "config_axes.h"

#include "esp_log.h"

static const char* TAG = "WIDTH_HDL";

extern "C" {

esp_err_t handle_width(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // Validate input
    if (cmd == nullptr || response == nullptr || resp_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if floating switch is initialized
    if (!floating_switch_is_initialized()) {
        ESP_LOGE(TAG, "Floating switch not initialized");
        return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
    }

    // AC7: WIDTH requires axis parameter, must be C
    if (cmd->axis == '\0') {
        // No axis specified - treat as C axis (only valid axis)
        ESP_LOGD(TAG, "No axis specified, defaulting to C");
    } else {
        // Validate axis character
        if (!is_valid_axis(cmd->axis)) {
            ESP_LOGD(TAG, "Invalid axis: %c", cmd->axis);
            return format_error(response, resp_len, ERR_INVALID_AXIS, MSG_INVALID_AXIS);
        }

        // Convert axis character to index
        int8_t axis_id = axis_to_index(cmd->axis);
        if (axis_id < 0) {
            return format_error(response, resp_len, ERR_INVALID_AXIS, MSG_INVALID_AXIS);
        }

        uint8_t axis_idx = static_cast<uint8_t>(axis_id);

        // AC7: Only C axis has floating switch
        if (!AXIS_HAS_FLOATING_SWITCH(axis_idx)) {
            ESP_LOGD(TAG, "Axis %c has no floating switch", cmd->axis);
            return format_error(response, resp_len, ERR_WIDTH_C_ONLY, MSG_WIDTH_C_ONLY);
        }
    }

    // AC4: Get last measured width
    float width = floating_switch_get_width();

    ESP_LOGD(TAG, "WIDTH C = %.3f", width);
    return format_ok_data(response, resp_len, "C %.3f", width);
}

esp_err_t width_handler_register(void)
{
    CommandEntry entry = {
        .verb = CMD_WIDTH,
        .handler = handle_width,
        .allowed_states = STATE_ANY  // Allow in any state
    };

    esp_err_t ret = cmd_executor_register(&entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered WIDTH handler");
    } else {
        ESP_LOGE(TAG, "Failed to register WIDTH handler: 0x%x", ret);
    }
    return ret;
}

}  // extern "C"
