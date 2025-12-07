/**
 * @file position_handler.cpp
 * @brief POS command handler implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 3-10 implementation - AC5-AC7
 */

#include "position_handler.h"
#include "motion_controller.h"
#include "command_executor.h"
#include "command_parser.h"
#include "response_formatter.h"
#include "config_commands.h"
#include "config_limits.h"
#include "config_axes.h"
#include "config_defaults.h"

#include "esp_log.h"
#include <cstdio>
#include <cstring>

static const char* TAG = "POS_HDL";

extern "C" {

/**
 * @brief Format all axis positions (AC5)
 *
 * Format: "X:0.001000 Y:0.000000 Z:... A:... B:... C:... D:... E:..."
 */
static esp_err_t format_all_positions(MotionController* controller, char* response, size_t resp_len)
{
    // Start with "OK "
    int written = snprintf(response, resp_len, "%s ", RESP_OK);
    if (written < 0 || (size_t)written >= resp_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Buffer for position string - enough for one axis
    char pos_buf[32];
    const char axis_chars[] = {AXIS_CHAR_X, AXIS_CHAR_Y, AXIS_CHAR_Z, AXIS_CHAR_A,
                                AXIS_CHAR_B, AXIS_CHAR_C, AXIS_CHAR_D, AXIS_CHAR_E};

    // Append position for each axis
    for (uint8_t i = 0; i < LIMIT_NUM_AXES; i++) {
        IMotor* motor = controller->getMotor(i);
        if (motor == nullptr) {
            ESP_LOGE(TAG, "Motor %d is null", i);
            return ESP_ERR_INVALID_STATE;
        }

        float position = motor->getPosition();

        // Format: "X:0.001000" with space separator (except last)
        int pos_written;
        if (i < LIMIT_NUM_AXES - 1) {
            pos_written = snprintf(pos_buf, sizeof(pos_buf), "%c:" DEFAULT_POSITION_FMT " ",
                                   axis_chars[i], position);
        } else {
            // Last axis - no trailing space
            pos_written = snprintf(pos_buf, sizeof(pos_buf), "%c:" DEFAULT_POSITION_FMT,
                                   axis_chars[i], position);
        }

        if (pos_written < 0) {
            return ESP_ERR_INVALID_SIZE;
        }

        // Check if we have space in response buffer
        size_t remaining = resp_len - (size_t)written;
        if ((size_t)pos_written >= remaining) {
            ESP_LOGW(TAG, "Response buffer too small for all positions");
            return ESP_ERR_INVALID_SIZE;
        }

        // Append to response
        strncat(response + written, pos_buf, remaining - 1);
        written += pos_written;
    }

    // Append line ending
    size_t remaining = resp_len - (size_t)written;
    if (remaining < 3) {
        return ESP_ERR_INVALID_SIZE;
    }
    strncat(response + written, "\r\n", remaining - 1);

    return ESP_OK;
}

esp_err_t handle_position(const ParsedCommand* cmd, char* response, size_t resp_len)
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
        // No axis specified - return all positions (AC5)
        esp_err_t ret = format_all_positions(controller, response, resp_len);
        if (ret != ESP_OK) {
            return format_error(response, resp_len, ERR_COMMUNICATION, MSG_COMMUNICATION);
        }
        return ESP_OK;
    }

    // Single axis specified (AC6, AC7)
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

    // Get position and format response (AC6)
    float position = motor->getPosition();
    return format_ok_data(response, resp_len, DEFAULT_POSITION_FMT, position);
}

esp_err_t position_handler_register(void)
{
    CommandEntry entry = {
        .verb = CMD_POS,
        .handler = handle_position,
        .allowed_states = STATE_ANY  // POS allowed in any state (read-only query)
    };

    esp_err_t ret = cmd_executor_register(&entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered POS handler");
    } else {
        ESP_LOGE(TAG, "Failed to register POS handler: 0x%x", ret);
    }
    return ret;
}

}  // extern "C"
