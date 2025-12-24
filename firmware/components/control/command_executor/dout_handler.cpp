/**
 * @file dout_handler.cpp
 * @brief DOUT command handler implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-7: General-Purpose Digital I/O
 *       AC4: DOUT pin 1 sets output HIGH
 *       AC5: DOUT pin 0 sets output LOW
 *       AC6: DOUT query returns all outputs as binary
 *       AC7: DOUT alias sets output by alias name
 *       AC10: Invalid pin/alias returns error
 */

#include "dout_handler.h"
#include "digital_io.h"
#include "command_executor.h"
#include "response_formatter.h"
#include "config_commands.h"

#include "esp_log.h"
#include <cstdlib>
#include <cstring>
#include <cctype>

static const char* TAG = "DOUT_HDL";

extern "C" {

/**
 * @brief Check if string is a valid output pin number (0-7)
 */
static bool is_pin_number(const char* str, uint8_t* pin)
{
    if (str == NULL || str[0] == '\0') {
        return false;
    }

    // Check if it's a single digit 0-7
    if (str[1] == '\0' && str[0] >= '0' && str[0] <= '7') {
        *pin = str[0] - '0';
        return true;
    }

    // Try parsing as integer
    char* endptr;
    long val = strtol(str, &endptr, 10);
    if (*endptr == '\0' && val >= 0 && val <= 7) {
        *pin = static_cast<uint8_t>(val);
        return true;
    }

    return false;
}

/**
 * @brief Parse pin/alias and value from command string
 *
 * Command format: "DOUT [pin|alias] [value]"
 * str_param contains everything after "DOUT " e.g., "5 1" or "LIGHT 1"
 */
static bool parse_dout_params(const ParsedCommand* cmd, uint8_t* pin, bool* value, bool* is_alias, char* alias_name, size_t alias_len)
{
    *is_alias = false;

    // If we have numeric params, use them
    if (cmd->param_count >= 2) {
        // DOUT 5 1 format
        long pin_val = static_cast<long>(cmd->params[0]);
        long level = static_cast<long>(cmd->params[1]);

        if (pin_val < 0 || pin_val > 7) {
            return false;
        }
        *pin = static_cast<uint8_t>(pin_val);
        *value = (level != 0);
        return true;
    }

    // String parameter - need to parse "PIN VALUE" or "ALIAS VALUE"
    if (!cmd->has_str_param || cmd->str_param[0] == '\0') {
        return false;
    }

    // Copy to local buffer for parsing
    char buf[64];
    strncpy(buf, cmd->str_param, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    // Split into two tokens
    char* token1 = buf;
    char* token2 = nullptr;

    // Find space separator
    for (char* p = buf; *p != '\0'; p++) {
        if (*p == ' ' || *p == '\t') {
            *p = '\0';
            token2 = p + 1;
            // Skip additional whitespace
            while (*token2 == ' ' || *token2 == '\t') {
                token2++;
            }
            break;
        }
    }

    if (token1[0] == '\0' || token2 == nullptr || token2[0] == '\0') {
        return false;
    }

    // Parse value (must be 0 or 1)
    char* endptr;
    long level = strtol(token2, &endptr, 10);
    if (*endptr != '\0' || (level != 0 && level != 1)) {
        return false;
    }
    *value = (level != 0);

    // Try to parse first token as pin number
    if (is_pin_number(token1, pin)) {
        return true;
    }

    // Must be an alias
    *is_alias = true;
    strncpy(alias_name, token1, alias_len - 1);
    alias_name[alias_len - 1] = '\0';
    return true;
}

esp_err_t handle_dout(const ParsedCommand* cmd, char* response, size_t resp_len)
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

    // AC6: No parameter - query all outputs
    if (!cmd->has_str_param && cmd->param_count == 0) {
        uint8_t all_outputs = digital_io_get_all_outputs();

        // Format as 8-bit binary: "OK 0b00100000" (DOUT7:...:DOUT0)
        char binary_str[11];  // "0b" + 8 bits + null
        binary_str[0] = '0';
        binary_str[1] = 'b';
        for (int i = 7; i >= 0; i--) {
            binary_str[2 + (7 - i)] = (all_outputs & (1 << i)) ? '1' : '0';
        }
        binary_str[10] = '\0';

        ESP_LOGD(TAG, "DOUT all: %s (0x%02X)", binary_str, all_outputs);
        return format_ok_data(response, resp_len, "%s", binary_str);
    }

    // Parse pin/alias and value
    uint8_t pin;
    bool value;
    bool is_alias;
    char alias_name[32];

    if (!parse_dout_params(cmd, &pin, &value, &is_alias, alias_name, sizeof(alias_name))) {
        // Check if this is an invalid pin number
        if (cmd->param_count >= 1) {
            long pin_val = static_cast<long>(cmd->params[0]);
            if (pin_val > 7) {
                ESP_LOGD(TAG, "Invalid pin number: %ld (> 7)", pin_val);
                return format_error(response, resp_len, ERR_INVALID_PIN, MSG_PIN_OUT_OF_RANGE);
            }
        }
        return format_error(response, resp_len, ERR_INVALID_PARAMETER, MSG_INVALID_PARAMETER);
    }

    // Resolve alias if needed
    if (is_alias) {
        dio_type_t type;
        esp_err_t ret = digital_io_resolve_alias(alias_name, &type, &pin);
        if (ret != ESP_OK) {
            ESP_LOGD(TAG, "Unknown alias: %s", alias_name);
            return format_error(response, resp_len, ERR_UNKNOWN_ALIAS, MSG_ALIAS_NOT_FOUND);
        }

        // Check it's an output alias
        if (type != DIO_TYPE_OUTPUT) {
            ESP_LOGD(TAG, "Alias '%s' is not an output", alias_name);
            return format_error(response, resp_len, ERR_INVALID_PIN, MSG_PIN_OUT_OF_RANGE);
        }
    }

    // AC10: Validate pin is in range
    if (pin > 7) {
        ESP_LOGD(TAG, "Invalid pin number: %d (> 7)", pin);
        return format_error(response, resp_len, ERR_INVALID_PIN, MSG_PIN_OUT_OF_RANGE);
    }

    // AC4/AC5/AC7: Set the output
    esp_err_t ret = digital_io_set_output(pin, value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set DOUT%d: %s", pin, esp_err_to_name(ret));
        return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
    }

    if (is_alias) {
        ESP_LOGD(TAG, "DOUT %s (pin %d) = %d", alias_name, pin, value ? 1 : 0);
    } else {
        ESP_LOGD(TAG, "DOUT%d = %d", pin, value ? 1 : 0);
    }

    return format_ok(response, resp_len);
}

esp_err_t dout_handler_register(void)
{
    CommandEntry entry = {
        .verb = CMD_DOUT,
        .handler = handle_dout,
        .allowed_states = STATE_ANY  // Allow in any state
    };

    esp_err_t ret = cmd_executor_register(&entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered DOUT handler");
    } else {
        ESP_LOGE(TAG, "Failed to register DOUT handler: 0x%x", ret);
    }
    return ret;
}

}  // extern "C"
