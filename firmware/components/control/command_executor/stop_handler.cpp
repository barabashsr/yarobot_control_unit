/**
 * @file stop_handler.cpp
 * @brief STOP command handler implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 3-10 implementation - AC13-AC16
 */

#include "stop_handler.h"
#include "motion_controller.h"
#include "command_executor.h"
#include "command_parser.h"
#include "response_formatter.h"
#include "config_commands.h"
#include "config_limits.h"

#include "esp_log.h"

static const char* TAG = "STOP_HDL";

extern "C" {

/**
 * @brief Stop all moving axes (AC14)
 */
static esp_err_t stop_all_axes(MotionController* controller)
{
    esp_err_t result = ESP_OK;

    for (uint8_t i = 0; i < LIMIT_NUM_AXES; i++) {
        IMotor* motor = controller->getMotor(i);
        if (motor == nullptr) {
            continue;  // Skip null motors (shouldn't happen)
        }

        // Only stop if moving
        if (motor->isMoving()) {
            esp_err_t ret = motor->stop();
            if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                // Log error but continue stopping other axes
                ESP_LOGW(TAG, "Failed to stop axis %d: 0x%x", i, ret);
                result = ret;  // Return last error, but continue
            } else {
                ESP_LOGD(TAG, "Stopped axis %d", i);
            }
        }
    }

    return result;
}

esp_err_t handle_stop(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // Validate input
    if (cmd == nullptr || response == nullptr || resp_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Get motion controller
    MotionController* controller = getMotionController();
    if (controller == nullptr || !controller->isInitialized()) {
        ESP_LOGE(TAG, "Motion controller not initialized");
        return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
    }

    // Check if axis is specified
    if (cmd->axis == '\0') {
        // No axis - stop all moving axes (AC14)
        esp_err_t ret = stop_all_axes(controller);
        // Always return OK for STOP (even if some axes failed to stop)
        // This is safer - user expects motion to stop
        (void)ret;
        return format_ok(response, resp_len);
    }

    // Single axis specified (AC13, AC15, AC16)
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

    // Get motor
    IMotor* motor = controller->getMotor(static_cast<uint8_t>(axis_id));
    if (motor == nullptr) {
        ESP_LOGE(TAG, "Motor %d is null", axis_id);
        return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
    }

    // Execute stop (AC13, AC15)
    // - AC13: Controlled deceleration if moving
    // - AC15: OK returned even if already stopped
    esp_err_t ret = motor->stop();

    // Return OK even if motor wasn't moving (AC15)
    // ESP_ERR_INVALID_STATE means "not moving" which is fine
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGD(TAG, "Axis %c stop command accepted", cmd->axis);
        return format_ok(response, resp_len);
    } else {
        // Unexpected error
        ESP_LOGW(TAG, "Unexpected error from stop: 0x%x", ret);
        // Still return OK as stop is safety-critical
        return format_ok(response, resp_len);
    }
}

esp_err_t stop_handler_register(void)
{
    CommandEntry entry = {
        .verb = CMD_STOP,
        .handler = handle_stop,
        .allowed_states = STATE_ANY  // STOP allowed in any state for safety
    };

    esp_err_t ret = cmd_executor_register(&entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered STOP handler");
    } else {
        ESP_LOGE(TAG, "Failed to register STOP handler: 0x%x", ret);
    }
    return ret;
}

}  // extern "C"
