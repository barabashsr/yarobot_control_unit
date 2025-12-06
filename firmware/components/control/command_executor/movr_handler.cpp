/**
 * @file movr_handler.cpp
 * @brief MOVR command handler implementation
 * @author YaRobot Team
 * @date 2025
 */

#include "movr_handler.h"
#include "motion_controller.h"
#include "command_executor.h"
#include "command_parser.h"
#include "response_formatter.h"
#include "config_commands.h"
#include "config_limits.h"

#include "esp_log.h"

static const char* TAG = "MOVR_HDL";

extern "C" {

esp_err_t handle_movr(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // Validate input
    if (cmd == nullptr || response == nullptr || resp_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // MOVR requires axis (AC12)
    if (cmd->axis == '\0') {
        ESP_LOGD(TAG, "No axis specified");
        return format_error(response, resp_len, ERR_INVALID_AXIS, MSG_INVALID_AXIS);
    }

    // Validate axis character
    if (!is_valid_axis(cmd->axis)) {
        ESP_LOGD(TAG, "Invalid axis: %c", cmd->axis);
        return format_error(response, resp_len, ERR_INVALID_AXIS, MSG_INVALID_AXIS);
    }

    // MOVR requires at least 1 parameter (delta)
    if (cmd->param_count < 1) {
        ESP_LOGD(TAG, "No delta parameter");
        return format_error(response, resp_len, ERR_INVALID_PARAMETER, MSG_INVALID_PARAMETER);
    }

    // Extract delta (required)
    float delta = cmd->params[0];

    // Extract velocity (optional - 0 means use default)
    float velocity = 0.0f;
    if (cmd->param_count >= 2) {
        velocity = cmd->params[1];
    }

    // Convert axis character to index
    int8_t axis_id = axis_to_index(cmd->axis);
    if (axis_id < 0 || axis_id >= LIMIT_NUM_AXES) {
        ESP_LOGD(TAG, "Invalid axis index: %d", axis_id);
        return format_error(response, resp_len, ERR_INVALID_AXIS, MSG_INVALID_AXIS);
    }

    // Get motion controller
    MotionController* controller = getMotionController();
    if (controller == nullptr || !controller->isInitialized()) {
        ESP_LOGE(TAG, "Motion controller not initialized");
        return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
    }

    // Execute relative move (AC3, AC4)
    esp_err_t ret = controller->moveRelative(static_cast<uint8_t>(axis_id), delta, velocity);

    // Map return codes to responses
    if (ret == ESP_OK) {
        return format_ok(response, resp_len);
    } else if (ret == ESP_ERR_INVALID_STATE) {
        // Axis not enabled
        return format_error(response, resp_len, ERR_AXIS_NOT_ENABLED, MSG_AXIS_NOT_ENABLED);
    } else if (ret == ESP_ERR_INVALID_ARG) {
        // Position limit exceeded
        return format_error(response, resp_len, ERR_POSITION_LIMIT, MSG_POSITION_LIMIT);
    } else {
        // Unknown error
        ESP_LOGW(TAG, "Unexpected error from moveRelative: 0x%x", ret);
        return format_error(response, resp_len, ERR_COMMUNICATION, MSG_COMMUNICATION);
    }
}

esp_err_t movr_handler_register(void)
{
    CommandEntry entry = {
        .verb = CMD_MOVR,
        .handler = handle_movr,
        .allowed_states = STATE_READY  // Only allow in READY state
    };

    esp_err_t ret = cmd_executor_register(&entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered MOVR handler");
    } else {
        ESP_LOGE(TAG, "Failed to register MOVR handler: 0x%x", ret);
    }
    return ret;
}

}  // extern "C"
