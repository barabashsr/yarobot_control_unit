/**
 * @file enable_handler.cpp
 * @brief EN command handler implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 3-10 implementation - AC1-AC4
 */

#include "enable_handler.h"
#include "motion_controller.h"
#include "command_executor.h"
#include "command_parser.h"
#include "response_formatter.h"
#include "config_commands.h"
#include "config_limits.h"

#include "esp_log.h"

static const char* TAG = "EN_HDL";

extern "C" {

esp_err_t handle_enable(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // Validate input
    if (cmd == nullptr || response == nullptr || resp_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // EN requires axis (AC4)
    if (cmd->axis == '\0') {
        ESP_LOGD(TAG, "No axis specified");
        return format_error(response, resp_len, ERR_INVALID_AXIS, MSG_INVALID_AXIS);
    }

    // Validate axis character
    if (!is_valid_axis(cmd->axis)) {
        ESP_LOGD(TAG, "Invalid axis: %c", cmd->axis);
        return format_error(response, resp_len, ERR_INVALID_AXIS, MSG_INVALID_AXIS);
    }

    // EN requires 1 parameter (enable state: 0 or 1)
    if (cmd->param_count < 1) {
        ESP_LOGD(TAG, "No enable state parameter");
        return format_error(response, resp_len, ERR_INVALID_PARAMETER, MSG_INVALID_PARAMETER);
    }

    // Extract enable state (0 or 1)
    float enable_param = cmd->params[0];
    bool enable;
    if (enable_param == 0.0f) {
        enable = false;
    } else if (enable_param == 1.0f) {
        enable = true;
    } else {
        ESP_LOGD(TAG, "Invalid enable state: %.1f (must be 0 or 1)", enable_param);
        return format_error(response, resp_len, ERR_INVALID_PARAMETER, MSG_INVALID_PARAMETER);
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

    // Get motor
    IMotor* motor = controller->getMotor(static_cast<uint8_t>(axis_id));
    if (motor == nullptr) {
        ESP_LOGE(TAG, "Motor %d is null", axis_id);
        return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
    }

    // Execute enable/disable (AC1, AC2, AC3)
    // When disabling (enable=false):
    //   - If moving, motor->enable(false) stops immediately (AC2)
    //   - If idle, motor->enable(false) just disables (AC3)
    // When enabling (enable=true):
    //   - State changes from DISABLED to IDLE (AC1)
    esp_err_t ret = motor->enable(enable);

    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Axis %c %s", cmd->axis, enable ? "enabled" : "disabled");
        return format_ok(response, resp_len);
    } else if (ret == ESP_ERR_TIMEOUT) {
        // Shift register update timeout
        ESP_LOGW(TAG, "Enable timeout for axis %c", cmd->axis);
        return format_error(response, resp_len, ERR_COMMUNICATION, MSG_COMMUNICATION);
    } else {
        // Unknown error
        ESP_LOGW(TAG, "Unexpected error from enable: 0x%x", ret);
        return format_error(response, resp_len, ERR_COMMUNICATION, MSG_COMMUNICATION);
    }
}

esp_err_t enable_handler_register(void)
{
    CommandEntry entry = {
        .verb = CMD_EN,
        .handler = handle_enable,
        .allowed_states = STATE_ANY  // EN allowed in any state for safety
    };

    esp_err_t ret = cmd_executor_register(&entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered EN handler");
    } else {
        ESP_LOGE(TAG, "Failed to register EN handler: 0x%x", ret);
    }
    return ret;
}

}  // extern "C"
