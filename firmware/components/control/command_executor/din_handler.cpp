/**
 * @file din_handler.cpp
 * @brief DIN command handler implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-7: General-Purpose Digital I/O
 *       AC1: DIN reads all 4 inputs as binary
 *       AC2: DIN pin reads single input
 *       AC3: DIN alias reads by alias name
 *       AC10: Invalid pin/alias returns error
 */

#include "din_handler.h"
#include "digital_io.h"
#include "command_executor.h"
#include "response_formatter.h"
#include "config_commands.h"

#include "esp_log.h"
#include <cstdlib>
#include <cstring>
#include <cctype>

static const char* TAG = "DIN_HDL";

extern "C" {

/**
 * @brief Check if string is a valid pin number (0-3)
 */
static bool is_pin_number(const char* str, uint8_t* pin)
{
    if (str == NULL || str[0] == '\0') {
        return false;
    }

    // Check if it's a single digit 0-3
    if (str[1] == '\0' && str[0] >= '0' && str[0] <= '3') {
        *pin = str[0] - '0';
        return true;
    }

    // Try parsing as integer
    char* endptr;
    long val = strtol(str, &endptr, 10);
    if (*endptr == '\0' && val >= 0 && val <= 3) {
        *pin = static_cast<uint8_t>(val);
        return true;
    }

    return false;
}

esp_err_t handle_din(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // Validate input
    if (cmd == nullptr || response == nullptr || resp_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if digital I/O manager is initialized
    if (!digital_io_is_initialized()) {
        ESP_LOGE(TAG, "Digital I/O not initialized");
        return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
    }

    // AC1: No parameter - read all inputs
    if (!cmd->has_str_param && cmd->param_count == 0) {
        uint8_t all_inputs = digital_io_read_all_inputs();

        // Format as binary: "OK 0b1010" (DIN3:DIN2:DIN1:DIN0)
        char binary_str[7];  // "0b" + 4 bits + null
        binary_str[0] = '0';
        binary_str[1] = 'b';
        binary_str[2] = (all_inputs & 0x08) ? '1' : '0';  // DIN3
        binary_str[3] = (all_inputs & 0x04) ? '1' : '0';  // DIN2
        binary_str[4] = (all_inputs & 0x02) ? '1' : '0';  // DIN1
        binary_str[5] = (all_inputs & 0x01) ? '1' : '0';  // DIN0
        binary_str[6] = '\0';

        ESP_LOGD(TAG, "DIN all: %s (0x%02X)", binary_str, all_inputs);
        return format_ok_data(response, resp_len, "%s", binary_str);
    }

    // Has parameter - either pin number or alias
    const char* param = nullptr;
    char param_buf[32];

    if (cmd->has_str_param) {
        param = cmd->str_param;
    } else if (cmd->param_count > 0) {
        // Numeric parameter - convert to string
        snprintf(param_buf, sizeof(param_buf), "%d", static_cast<int>(cmd->params[0]));
        param = param_buf;
    }

    if (param == nullptr) {
        return format_error(response, resp_len, ERR_INVALID_PARAMETER, MSG_INVALID_PARAMETER);
    }

    // AC2: Try to parse as pin number (0-3)
    uint8_t pin;
    if (is_pin_number(param, &pin)) {
        bool value;
        esp_err_t ret = digital_io_read_input(pin, &value);
        if (ret != ESP_OK) {
            // Pin out of range (shouldn't happen since we validated 0-3)
            return format_error(response, resp_len, ERR_INVALID_PIN, MSG_PIN_OUT_OF_RANGE);
        }

        ESP_LOGD(TAG, "DIN%d = %d", pin, value ? 1 : 0);
        return format_ok_data(response, resp_len, "DIN%d %d", pin, value ? 1 : 0);
    }

    // AC3: Try to resolve as alias
    dio_type_t type;
    esp_err_t ret = digital_io_resolve_alias(param, &type, &pin);
    if (ret == ESP_OK) {
        // Check it's an input alias
        if (type != DIO_TYPE_INPUT) {
            ESP_LOGD(TAG, "Alias '%s' is not an input", param);
            return format_error(response, resp_len, ERR_INVALID_PIN, MSG_PIN_OUT_OF_RANGE);
        }

        bool value;
        ret = digital_io_read_input(pin, &value);
        if (ret != ESP_OK) {
            return format_error(response, resp_len, ERR_INVALID_PIN, MSG_PIN_OUT_OF_RANGE);
        }

        ESP_LOGD(TAG, "DIN %s (pin %d) = %d", param, pin, value ? 1 : 0);
        return format_ok_data(response, resp_len, "%s %d", param, value ? 1 : 0);
    }

    // AC10: Invalid pin number (> 3) check
    char* endptr;
    long val = strtol(param, &endptr, 10);
    if (*endptr == '\0' && val > 3) {
        ESP_LOGD(TAG, "Invalid pin number: %ld (> 3)", val);
        return format_error(response, resp_len, ERR_INVALID_PIN, MSG_PIN_OUT_OF_RANGE);
    }

    // AC10: Unknown alias
    ESP_LOGD(TAG, "Unknown alias: %s", param);
    return format_error(response, resp_len, ERR_UNKNOWN_ALIAS, MSG_ALIAS_NOT_FOUND);
}

esp_err_t din_handler_register(void)
{
    CommandEntry entry = {
        .verb = CMD_DIN,
        .handler = handle_din,
        .allowed_states = STATE_ANY  // Allow in any state
    };

    esp_err_t ret = cmd_executor_register(&entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered DIN handler");
    } else {
        ESP_LOGE(TAG, "Failed to register DIN handler: 0x%x", ret);
    }
    return ret;
}

}  // extern "C"
