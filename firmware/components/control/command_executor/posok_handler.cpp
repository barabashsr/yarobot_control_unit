/**
 * @file posok_handler.cpp
 * @brief POSOK command handler implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-6: Position Loss Detection
 *       AC4: POSOK X clears POSLOS for single axis, returns to IDLE
 *       AC5: POSOK clears POSLOS for all servo axes
 */

#include "posok_handler.h"
#include "position_loss.h"
#include "command_executor.h"
#include "command_parser.h"
#include "response_formatter.h"
#include "config_commands.h"
#include "config_limits.h"

#include "esp_log.h"

static const char* TAG = "POSOK_HDL";

extern "C" {

esp_err_t handle_posok(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // Validate input
    if (cmd == nullptr || response == nullptr || resp_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if position loss detector is initialized
    if (!position_loss_is_initialized()) {
        ESP_LOGE(TAG, "Position loss detector not initialized");
        return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
    }

    // AC5: No axis specified - clear all servo axes
    if (cmd->axis == '\0') {
        esp_err_t ret = position_loss_acknowledge_all();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to acknowledge all axes: %s", esp_err_to_name(ret));
            return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
        }
        ESP_LOGI(TAG, "POSOK: All servo axes acknowledged");
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

    // AC9: Check if axis is a servo (steppers don't have POSLOS)
    if (!position_loss_is_servo_axis(axis_idx)) {
        ESP_LOGD(TAG, "Axis %c is not a servo axis (use HOME for steppers)", cmd->axis);
        return format_error(response, resp_len, ERR_INVALID_AXIS,
                            "Stepper axis - use HOME");
    }

    // AC4: Clear POSLOS for single axis
    esp_err_t ret = position_loss_acknowledge(axis_idx);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_INVALID_STATE) {
            // Axis not in POSLOS state - this is actually OK per story
            // User can call POSOK even if not POSLOS
            ESP_LOGD(TAG, "Axis %c not in POSLOS state (already OK)", cmd->axis);
        } else {
            ESP_LOGW(TAG, "Failed to acknowledge axis %c: %s", cmd->axis, esp_err_to_name(ret));
            return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
        }
    }

    ESP_LOGI(TAG, "POSOK: Axis %c acknowledged", cmd->axis);
    return format_ok(response, resp_len);
}

esp_err_t posok_handler_register(void)
{
    CommandEntry entry = {
        .verb = CMD_POSOK,
        .handler = handle_posok,
        .allowed_states = STATE_IDLE | STATE_READY  // Allow in IDLE and READY states
    };

    esp_err_t ret = cmd_executor_register(&entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered POSOK handler");
    } else {
        ESP_LOGE(TAG, "Failed to register POSOK handler: 0x%x", ret);
    }
    return ret;
}

}  // extern "C"
