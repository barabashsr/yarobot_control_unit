/**
 * @file brake_handler.cpp
 * @brief BRAKE command handler implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-5: Brake Control System
 *       - BRAKE <axis>       - Query brake state (returns ENGAGED/RELEASED)
 *       - BRAKE <axis> <0|1> - Manual brake control (0=engage, 1=release)
 *       - Set command only allowed for BRAKE_MANUAL strategy
 *       - Other strategies return ERR_BRAKE_AUTO for set commands
 *       - Axes without brake hardware (C, D, E) return ERR_AXIS_NO_BRAKE
 */

#include "brake_handler.h"
#include "brake_controller.h"
#include "command_executor.h"
#include "command_parser.h"
#include "response_formatter.h"
#include "config_commands.h"
#include "config_limits.h"

#include "esp_log.h"

static const char* TAG = "BRAKE_HDL";

extern "C" {

esp_err_t handle_brake(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // Validate input
    if (cmd == nullptr || response == nullptr || resp_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // BRAKE requires axis
    if (cmd->axis == '\0') {
        ESP_LOGD(TAG, "No axis specified");
        return format_error(response, resp_len, ERR_INVALID_AXIS, MSG_INVALID_AXIS);
    }

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

    // Check if axis has brake hardware (X, Y, Z, A, B only)
    if (!brake_has_hardware(axis_idx)) {
        ESP_LOGD(TAG, "Axis %c has no brake hardware", cmd->axis);
        return format_error(response, resp_len, ERR_AXIS_NO_BRAKE, MSG_AXIS_NO_BRAKE);
    }

    // Check if brake controller is initialized
    if (!brake_controller_is_initialized()) {
        ESP_LOGE(TAG, "Brake controller not initialized");
        return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
    }

    // Query mode: BRAKE <axis> with no parameter returns current state
    if (cmd->param_count < 1) {
        bool engaged = brake_get_state(axis_idx);
        return format_ok_data(response, resp_len, "%c %s",
                              cmd->axis, engaged ? "ENGAGED" : "RELEASED");
    }

    // Extract brake state (0 or 1)
    float brake_param = cmd->params[0];
    bool release;
    if (brake_param == 0.0f) {
        release = false;  // Engage brake
    } else if (brake_param == 1.0f) {
        release = true;   // Release brake
    } else {
        ESP_LOGD(TAG, "Invalid brake state: %.1f (must be 0 or 1)", brake_param);
        return format_error(response, resp_len, ERR_INVALID_PARAMETER, MSG_INVALID_PARAMETER);
    }

    // Check brake strategy - only BRAKE_MANUAL allows manual control
    BrakeStrategy strategy = brake_get_strategy(axis_idx);
    if (strategy != BRAKE_MANUAL) {
        ESP_LOGD(TAG, "Axis %c brake is auto-managed (strategy=%d)", cmd->axis, strategy);
        return format_error(response, resp_len, ERR_BRAKE_AUTO, MSG_BRAKE_AUTO);
    }

    // Execute brake control with timing
    esp_err_t ret = brake_set_state_with_timing(axis_idx, !release);  // !release because true=engage

    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Axis %c brake %s", cmd->axis, release ? "released" : "engaged");
        return format_ok(response, resp_len);
    } else if (ret == ESP_ERR_INVALID_ARG) {
        ESP_LOGW(TAG, "Invalid axis for brake control");
        return format_error(response, resp_len, ERR_INVALID_AXIS, MSG_INVALID_AXIS);
    } else {
        ESP_LOGW(TAG, "Unexpected error from brake control: 0x%x", ret);
        return format_error(response, resp_len, ERR_COMMUNICATION, MSG_COMMUNICATION);
    }
}

esp_err_t brake_handler_register(void)
{
    CommandEntry entry = {
        .verb = CMD_BRAKE,
        .handler = handle_brake,
        .allowed_states = STATE_ANY  // BRAKE allowed in any state
    };

    esp_err_t ret = cmd_executor_register(&entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered BRAKE handler");
    } else {
        ESP_LOGE(TAG, "Failed to register BRAKE handler: 0x%x", ret);
    }
    return ret;
}

}  // extern "C"
