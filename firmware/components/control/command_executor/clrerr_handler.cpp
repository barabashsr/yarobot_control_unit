/**
 * @file clrerr_handler.cpp
 * @brief CLRERR command handler implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-10: Error Tracking & Recovery
 *       AC4: CLRERR X clears error state for single axis
 *       AC4: CLRERR clears error state for all axes
 */

#include "clrerr_handler.h"
#include "error_manager.h"
#include "command_executor.h"
#include "command_parser.h"
#include "response_formatter.h"
#include "config_commands.h"
#include "config_limits.h"

#include "esp_log.h"

static const char* TAG = "CLRERR_HDL";

extern "C" {

esp_err_t handle_clrerr(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // Validate input
    if (cmd == nullptr || response == nullptr || resp_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if error manager is initialized
    if (!error_manager_is_initialized()) {
        ESP_LOGE(TAG, "Error manager not initialized");
        return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
    }

    // AC4: No axis specified - clear all axis errors
    if (cmd->axis == '\0') {
        esp_err_t ret = error_manager_clear_all_axis_errors();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to clear all axis errors: %s", esp_err_to_name(ret));
            return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
        }
        ESP_LOGI(TAG, "CLRERR: All axis error states cleared");
        return format_ok(response, resp_len);
    }

    // AC4: Single axis specified
    // Validate axis character
    if (!is_valid_axis(cmd->axis)) {
        ESP_LOGD(TAG, "Invalid axis: %c", cmd->axis);
        return format_error(response, resp_len, ERR_INVALID_AXIS, MSG_INVALID_AXIS);
    }

    // Convert axis character to index
    int8_t axis_id = axis_to_index(cmd->axis);
    if (axis_id < 0 || axis_id >= LIMIT_NUM_AXES) {
        ESP_LOGD(TAG, "Invalid axis index: %d", axis_id);
        return format_error(response, resp_len, ERR_INVALID_AXIS, MSG_INVALID_AXIS);
    }

    uint8_t axis_idx = static_cast<uint8_t>(axis_id);

    // Clear error state for single axis
    esp_err_t ret = error_manager_clear_axis_error(axis_idx);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear axis %c error: %s", cmd->axis, esp_err_to_name(ret));
        return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
    }

    ESP_LOGI(TAG, "CLRERR: Axis %c error state cleared", cmd->axis);
    return format_ok(response, resp_len);
}

esp_err_t clrerr_handler_register(void)
{
    CommandEntry entry = {
        .verb = CMD_CLRERR,
        .handler = handle_clrerr,
        .allowed_states = STATE_IDLE | STATE_READY | STATE_ERROR  // Allow in all operational states
    };

    esp_err_t ret = cmd_executor_register(&entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered CLRERR handler");
    } else {
        ESP_LOGE(TAG, "Failed to register CLRERR handler: 0x%x", ret);
    }
    return ret;
}

}  // extern "C"
