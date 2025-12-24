/**
 * @file i2c_handler.cpp
 * @brief I2C command handler implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-11: I2C Communication Health
 *       AC5: I2C command returns health status for all tracked devices
 *       AC6: I2C SCAN scans bus and returns found addresses
 */

#include "i2c_handler.h"
#include "i2c_health_monitor.h"
#include "i2c_hal.h"
#include "command_executor.h"
#include "command_parser.h"
#include "response_formatter.h"
#include "config_commands.h"
#include "config_i2c.h"

#include "esp_log.h"
#include <cstring>
#include <cstdio>

static const char* TAG = "I2C_HDL";

extern "C" {

esp_err_t handle_i2c(const ParsedCommand* cmd, char* response, size_t resp_len)
{
    // Validate input
    if (cmd == nullptr || response == nullptr || resp_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check for SCAN argument
    if (cmd->has_str_param && strcasecmp(cmd->str_param, "SCAN") == 0) {
        // I2C SCAN: Scan bus and return found addresses
        if (!i2c_hal_is_initialized(I2C_PORT)) {
            ESP_LOGW(TAG, "I2C bus not initialized");
            return format_error(response, resp_len, ERR_I2C_FAILURE, MSG_I2C_FAILURE);
        }

        uint8_t found_addrs[16];
        size_t count = 0;

        esp_err_t ret = i2c_hal_scan_bus(I2C_PORT, found_addrs, 16, &count);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "I2C scan failed: %s", esp_err_to_name(ret));
            return format_error(response, resp_len, ERR_I2C_FAILURE, MSG_I2C_FAILURE);
        }

        // Format response: "OK I2C SCAN 0x20 0x21 ..."
        int written = snprintf(response, resp_len, "OK I2C SCAN");
        for (size_t i = 0; i < count && written < (int)resp_len - 6; i++) {
            written += snprintf(response + written, resp_len - written, " 0x%02X", found_addrs[i]);
        }

        if (count == 0) {
            strncat(response, " (none)", resp_len - strlen(response) - 1);
        }

        ESP_LOGI(TAG, "I2C SCAN: found %zu device(s)", count);
        return ESP_OK;
    }

    // I2C with no args: Return health status for all tracked devices
    if (!i2c_health_is_initialized()) {
        ESP_LOGW(TAG, "I2C health monitor not initialized");
        return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
    }

    // Get all tracked devices
    I2CDeviceHealth devices[I2C_MAX_DEVICES];
    size_t device_count = 0;

    esp_err_t ret = i2c_health_get_all_devices(devices, I2C_MAX_DEVICES, &device_count);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get device health: %s", esp_err_to_name(ret));
        return format_error(response, resp_len, ERR_CONFIGURATION, MSG_CONFIGURATION);
    }

    if (device_count == 0) {
        return format_ok_data(response, resp_len, "I2C (no devices tracked)");
    }

    // Format response: "OK I2C 0x20:ONLINE:100:0 0x21:OFFLINE:50:5"
    // Format per device: <addr>:<status>:<success>:<failures>
    int written = snprintf(response, resp_len, "OK I2C");

    for (size_t i = 0; i < device_count && written < (int)resp_len - 32; i++) {
        const char* status = devices[i].is_online ? "ONLINE" : "OFFLINE";
        written += snprintf(response + written, resp_len - written,
                            " 0x%02X:%s:%lu:%lu",
                            devices[i].address,
                            status,
                            (unsigned long)devices[i].transaction_count,
                            (unsigned long)devices[i].failure_count);
    }

    // Also add overall bus health
    I2CBusHealth bus_health;
    if (i2c_health_get_bus_status(I2C_PORT, &bus_health) == ESP_OK) {
        const char* bus_status;
        switch (bus_health.status) {
            case I2C_HEALTH_OK:    bus_status = "OK"; break;
            case I2C_HEALTH_WARN:  bus_status = "WARN"; break;
            case I2C_HEALTH_ERROR: bus_status = "ERROR"; break;
            default:               bus_status = "?"; break;
        }

        written += snprintf(response + written, resp_len - written,
                            " BUS:%s:%u/%u",
                            bus_status,
                            bus_health.online_device_count,
                            bus_health.total_device_count);
    }

    ESP_LOGI(TAG, "I2C: reported %zu device(s)", device_count);
    return ESP_OK;
}

esp_err_t i2c_handler_register(void)
{
    CommandEntry entry = {
        .verb = CMD_I2C,
        .handler = handle_i2c,
        .allowed_states = STATE_IDLE | STATE_READY | STATE_ERROR  // Allow in all operational states
    };

    esp_err_t ret = cmd_executor_register(&entry);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered I2C handler");
    } else {
        ESP_LOGE(TAG, "Failed to register I2C handler: 0x%x", ret);
    }
    return ret;
}

}  // extern "C"
