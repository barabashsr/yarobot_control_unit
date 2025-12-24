/**
 * @file errcnt_handler.cpp
 * @brief ERRCNT command handler implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-10: Error Tracking & Recovery
 *       AC1: ERRCNT returns "OK I2C:n TIMEOUT:n LIMIT:n ESTOP:n POSLOS:n"
 *       AC2: ERRCNT X returns "OK X LIMIT:n POSLOS:n TIMEOUT:n"
 *       AC7: ERRCNT CLR resets all error counts
 */

#include "errcnt_handler.h"
#include "error_manager.h"
#include "command_executor.h"
#include "command_parser.h"
#include "response_formatter.h"
#include "config_commands.h"
#include "config_limits.h"

#include "esp_log.h"
#include <cstring>
#include <cstdio>

static const char* TAG = "ERRCNT_HDL";

extern "C" {

esp_err_t handle_errcnt(const ParsedCommand* cmd, char* response, size_t resp_len)
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

    // Check for CLR argument (case-insensitive)
    if (cmd->has_str_param &&
        (strcasecmp(cmd->str_param, "CLR") == 0 || strcasecmp(cmd->str_param, "CLEAR") == 0)) {
        // AC7: Clear all error counts
        esp_err_t ret = error_manager_clear_counts();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to clear counts: %s", esp_err_to_name(ret));
            return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
        }
        ESP_LOGI(TAG, "ERRCNT CLR: All error counts cleared");
        return format_ok(response, resp_len);
    }

    // Check for axis-specific query
    if (cmd->axis != '\0') {
        // AC2: Per-axis error counts
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

        // Get per-axis counts
        uint32_t counts[ERR_CAT_COUNT];
        esp_err_t ret = error_manager_get_axis_counts(axis_idx, counts);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to get axis counts: %s", esp_err_to_name(ret));
            return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
        }

        // Format response: "OK X LIMIT:n POSLOS:n TIMEOUT:n"
        // Per-axis shows only relevant categories in a different order
        return format_ok_data(response, resp_len,
            "%c LIMIT:%lu POSLOS:%lu TIMEOUT:%lu",
            cmd->axis,
            (unsigned long)counts[ERR_CAT_LIMIT],
            (unsigned long)counts[ERR_CAT_POSLOS],
            (unsigned long)counts[ERR_CAT_TIMEOUT]);
    }

    // AC1: Global error counts (no axis specified)
    uint32_t counts[ERR_CAT_COUNT];
    esp_err_t ret = error_manager_get_counts(counts);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get global counts: %s", esp_err_to_name(ret));
        return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
    }

    // Format response: "OK I2C:n TIMEOUT:n LIMIT:n ESTOP:n POSLOS:n"
    return format_ok_data(response, resp_len,
        "I2C:%lu TIMEOUT:%lu LIMIT:%lu ESTOP:%lu POSLOS:%lu",
        (unsigned long)counts[ERR_CAT_I2C],
        (unsigned long)counts[ERR_CAT_TIMEOUT],
        (unsigned long)counts[ERR_CAT_LIMIT],
        (unsigned long)counts[ERR_CAT_ESTOP],
        (unsigned long)counts[ERR_CAT_POSLOS]);
}

esp_err_t errcnt_handler_register(void)
{
    CommandEntry entry = {
        .verb = CMD_ERRCNT,
        .handler = handle_errcnt,
        .allowed_states = STATE_IDLE | STATE_READY | STATE_ERROR  // Allow in all operational states
    };

    esp_err_t ret = cmd_executor_register(&entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered ERRCNT handler");
    } else {
        ESP_LOGE(TAG, "Failed to register ERRCNT handler: 0x%x", ret);
    }
    return ret;
}

}  // extern "C"
