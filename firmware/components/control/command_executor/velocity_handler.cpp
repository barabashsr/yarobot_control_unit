/**
 * @file velocity_handler.cpp
 * @brief VEL command handler implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 3-10 implementation - AC8-AC12
 */

#include "velocity_handler.h"
#include "motion_controller.h"
#include "command_executor.h"
#include "command_parser.h"
#include "response_formatter.h"
#include "config_commands.h"
#include "config_limits.h"

#include "esp_log.h"

static const char* TAG = "VEL_HDL";

extern "C" {

esp_err_t handle_velocity(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // Validate input
    if (cmd == nullptr || response == nullptr || resp_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // VEL requires axis
    if (cmd->axis == '\0') {
        ESP_LOGD(TAG, "No axis specified");
        return format_error(response, resp_len, ERR_INVALID_AXIS, MSG_INVALID_AXIS);
    }

    // Validate axis character
    if (!is_valid_axis(cmd->axis)) {
        ESP_LOGD(TAG, "Invalid axis: %c", cmd->axis);
        return format_error(response, resp_len, ERR_INVALID_AXIS, MSG_INVALID_AXIS);
    }

    // VEL requires 1 parameter (velocity)
    if (cmd->param_count < 1) {
        ESP_LOGD(TAG, "No velocity parameter");
        return format_error(response, resp_len, ERR_INVALID_PARAMETER, MSG_INVALID_PARAMETER);
    }

    // Extract velocity (signed value, SI units)
    float velocity = cmd->params[0];

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

    // Get motor
    IMotor* motor = controller->getMotor(static_cast<uint8_t>(axis_id));
    if (motor == nullptr) {
        ESP_LOGE(TAG, "Motor %d is null", axis_id);
        return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
    }

    // Check if motor is enabled (AC10)
    if (!motor->isEnabled()) {
        ESP_LOGD(TAG, "Axis %c not enabled", cmd->axis);
        return format_error(response, resp_len, ERR_AXIS_NOT_ENABLED, MSG_AXIS_NOT_ENABLED);
    }

    // Execute velocity mode (AC8, AC9, AC11, AC12)
    // - AC8: Continuous motion at target velocity
    // - AC9: Signed velocity determines direction
    // - AC11: Motor's moveVelocity() clamps to config.max_velocity
    // - AC12: New VEL during VEL motion blends to new target (motor handles this)
    esp_err_t ret = motor->moveVelocity(velocity);

    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Axis %c velocity mode: %.3f", cmd->axis, velocity);
        return format_ok(response, resp_len);
    } else if (ret == ESP_ERR_INVALID_STATE) {
        // Axis not enabled (shouldn't happen after check above, but handle it)
        return format_error(response, resp_len, ERR_AXIS_NOT_ENABLED, MSG_AXIS_NOT_ENABLED);
    } else {
        // Unknown error
        ESP_LOGW(TAG, "Unexpected error from moveVelocity: 0x%x", ret);
        return format_error(response, resp_len, ERR_COMMUNICATION, MSG_COMMUNICATION);
    }
}

esp_err_t velocity_handler_register(void)
{
    CommandEntry entry = {
        .verb = CMD_VEL,
        .handler = handle_velocity,
        .allowed_states = STATE_READY  // VEL requires READY state (motion allowed)
    };

    esp_err_t ret = cmd_executor_register(&entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered VEL handler");
    } else {
        ESP_LOGE(TAG, "Failed to register VEL handler: 0x%x", ret);
    }
    return ret;
}

}  // extern "C"
