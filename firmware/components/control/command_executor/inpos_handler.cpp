/**
 * @file inpos_handler.cpp
 * @brief INPOS command handler implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-8: Servo Feedback Processing (Basic InPos)
 *       AC2: INPOS reads all 5 servo axes
 *       AC3: INPOS X reads single servo axis
 *       AC5: INPOS C returns N/A for stepper axes
 *       AC6: Invalid axis returns error
 */

#include "inpos_handler.h"
#include "servo_feedback.h"
#include "command_executor.h"
#include "response_formatter.h"
#include "config_commands.h"
#include "config_axes.h"

#include "esp_log.h"

static const char* TAG = "INPOS_HDL";

extern "C" {

esp_err_t handle_inpos(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // Validate input
    if (cmd == nullptr || response == nullptr || resp_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if servo feedback is initialized
    if (!servo_feedback_is_initialized()) {
        ESP_LOGE(TAG, "Servo feedback not initialized");
        return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
    }

    // AC2: No axis parameter - read all servo axes
    if (cmd->axis == '\0') {
        uint8_t inpos_bitmap;
        esp_err_t ret = servo_feedback_get_all_inpos(&inpos_bitmap);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read InPos: %s", esp_err_to_name(ret));
            return format_error(response, resp_len, ERR_COMMUNICATION, MSG_COMMUNICATION);
        }

        // Format as "OK X:1 Y:0 Z:1 A:1 B:0"
        int x = (inpos_bitmap >> 0) & 1;
        int y = (inpos_bitmap >> 1) & 1;
        int z = (inpos_bitmap >> 2) & 1;
        int a = (inpos_bitmap >> 3) & 1;
        int b = (inpos_bitmap >> 4) & 1;

        ESP_LOGD(TAG, "INPOS all: X=%d Y=%d Z=%d A=%d B=%d", x, y, z, a, b);
        return format_ok_data(response, resp_len, "X:%d Y:%d Z:%d A:%d B:%d", x, y, z, a, b);
    }

    // AC6: Validate axis character
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

    // AC5: Stepper/discrete axes return N/A
    if (!AXIS_HAS_SERVO_FEEDBACK(axis_idx)) {
        ESP_LOGD(TAG, "Axis %c has no servo feedback", cmd->axis);
        return format_ok_data(response, resp_len, "%c N/A", cmd->axis);
    }

    // AC3: Read single servo axis
    bool inpos;
    esp_err_t ret = servo_feedback_get_inpos(axis_idx, &inpos);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read InPos for axis %c: %s", cmd->axis, esp_err_to_name(ret));
        return format_error(response, resp_len, ERR_COMMUNICATION, MSG_COMMUNICATION);
    }

    ESP_LOGD(TAG, "INPOS %c = %d", cmd->axis, inpos ? 1 : 0);
    return format_ok_data(response, resp_len, "%c %d", cmd->axis, inpos ? 1 : 0);
}

esp_err_t inpos_handler_register(void)
{
    CommandEntry entry = {
        .verb = CMD_INPOS,
        .handler = handle_inpos,
        .allowed_states = STATE_ANY  // Allow in any state
    };

    esp_err_t ret = cmd_executor_register(&entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered INPOS handler");
    } else {
        ESP_LOGE(TAG, "Failed to register INPOS handler: 0x%x", ret);
    }
    return ret;
}

}  // extern "C"
