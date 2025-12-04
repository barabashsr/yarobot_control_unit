/**
 * @file test_utils.c
 * @brief Hardware test utilities implementation
 * @author YaRobot Team
 * @date 2025
 */

#include "test_utils.h"
#include "i2c_hal.h"
#include "spi_hal.h"
#include "gpio_hal.h"
#include "config_i2c.h"
#include "config_oled.h"
#include "config_gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "test_utils";

/** @brief Test patterns for shift register (24-bit) */
static const uint32_t sr_test_patterns[] = {
    0xAAAAAA,  // Alternating 1s (10101010...)
    0x555555,  // Alternating 0s (01010101...)
    0x000000,  // All zeros
    0xFFFFFF,  // All ones
};

esp_err_t test_i2c(test_result_t *result)
{
    if (result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    result->passed = false;
    memset(result->message, 0, sizeof(result->message));

    uint8_t found_i2c0[16] = {0};
    uint8_t found_i2c1[16] = {0};
    size_t count_i2c0 = 0;
    size_t count_i2c1 = 0;

    bool mcp0_found = false;
    bool mcp1_found = false;
    bool oled_found = false;

    // Test I2C0 if initialized
    if (i2c_hal_is_initialized(I2C_PORT)) {
        esp_err_t ret = i2c_hal_scan_bus(I2C_PORT, found_i2c0, 16, &count_i2c0);
        if (ret == ESP_OK && count_i2c0 > 0) {
            for (size_t i = 0; i < count_i2c0; i++) {
                if (found_i2c0[i] == I2C_ADDR_MCP23017_0) mcp0_found = true;
                if (found_i2c0[i] == I2C_ADDR_MCP23017_1) mcp1_found = true;
            }
        }
    } else {
        ESP_LOGW(TAG, "I2C0 not initialized");
    }

    // Test I2C1 if initialized
    if (i2c_hal_is_initialized(I2C_OLED_PORT)) {
        esp_err_t ret = i2c_hal_scan_bus(I2C_OLED_PORT, found_i2c1, 16, &count_i2c1);
        if (ret == ESP_OK && count_i2c1 > 0) {
            for (size_t i = 0; i < count_i2c1; i++) {
                if (found_i2c1[i] == OLED_ADDRESS) oled_found = true;
            }
        }
    } else {
        ESP_LOGW(TAG, "I2C1 not initialized");
    }

    // Build result message
    char i2c0_str[48] = "UNINIT";
    char i2c1_str[32] = "UNINIT";

    if (i2c_hal_is_initialized(I2C_PORT)) {
        if (count_i2c0 > 0) {
            int pos = 0;
            for (size_t i = 0; i < count_i2c0 && pos < 40; i++) {
                pos += snprintf(i2c0_str + pos, sizeof(i2c0_str) - pos,
                               "%s0x%02X", i > 0 ? "," : "", found_i2c0[i]);
            }
        } else {
            snprintf(i2c0_str, sizeof(i2c0_str), "NO_DEVICES");
        }
    }

    if (i2c_hal_is_initialized(I2C_OLED_PORT)) {
        if (count_i2c1 > 0) {
            int pos = 0;
            for (size_t i = 0; i < count_i2c1 && pos < 24; i++) {
                pos += snprintf(i2c1_str + pos, sizeof(i2c1_str) - pos,
                               "%s0x%02X", i > 0 ? "," : "", found_i2c1[i]);
            }
        } else {
            snprintf(i2c1_str, sizeof(i2c1_str), "NO_DEVICES");
        }
    }

    // Determine pass/fail
    bool all_expected = mcp0_found && mcp1_found && oled_found;

    if (all_expected) {
        result->passed = true;
        snprintf(result->message, sizeof(result->message),
                 "OK I2C0:%s I2C1:%s", i2c0_str, i2c1_str);
    } else {
        result->passed = false;
        snprintf(result->message, sizeof(result->message),
                 "ERROR I2C0:%s I2C1:%s", i2c0_str, i2c1_str);
    }

    ESP_LOGI(TAG, "I2C test: %s", result->message);
    return ESP_OK;
}

esp_err_t test_sr(test_result_t *result)
{
    if (result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    result->passed = false;
    memset(result->message, 0, sizeof(result->message));

    if (!spi_hal_is_initialized()) {
        snprintf(result->message, sizeof(result->message), "ERROR SR:UNINIT");
        ESP_LOGW(TAG, "SPI not initialized for SR test");
        return ESP_OK;
    }

    bool all_ok = true;

    // Write test patterns
    for (size_t i = 0; i < sizeof(sr_test_patterns) / sizeof(sr_test_patterns[0]); i++) {
        esp_err_t ret = spi_hal_sr_write(sr_test_patterns[i], 24);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SR pattern 0x%06lX failed", (unsigned long)sr_test_patterns[i]);
            all_ok = false;
            break;
        }
        ESP_LOGI(TAG, "SR: Wrote pattern 0x%06lX", (unsigned long)sr_test_patterns[i]);
        vTaskDelay(pdMS_TO_TICKS(10));  // Allow settling
    }

    // Test OE toggle
    if (all_ok) {
        esp_err_t ret = spi_hal_sr_set_oe(true);  // Enable outputs
        if (ret != ESP_OK) all_ok = false;
        vTaskDelay(pdMS_TO_TICKS(10));

        ret = spi_hal_sr_set_oe(false);  // Disable outputs
        if (ret != ESP_OK) all_ok = false;
    }

    // Return to safe state (all zeros = brakes engaged, motors disabled)
    spi_hal_sr_write(0x000000, 24);

    if (all_ok) {
        result->passed = true;
        snprintf(result->message, sizeof(result->message), "OK SR:PASS");
    } else {
        result->passed = false;
        snprintf(result->message, sizeof(result->message), "ERROR SR:FAIL");
    }

    ESP_LOGI(TAG, "SR test: %s", result->message);
    return ESP_OK;
}

esp_err_t test_gpio(test_result_t *result)
{
    if (result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    result->passed = false;
    memset(result->message, 0, sizeof(result->message));

    // Read E-STOP state
    int estop = gpio_hal_get_level(GPIO_E_STOP);

    // Briefly toggle STEP pins (just to verify they're configured)
    // Note: This is a quick pulse test, actual step verification requires logic analyzer
    const gpio_num_t step_pins[] = {
        GPIO_X_STEP, GPIO_Y_STEP, GPIO_Z_STEP, GPIO_A_STEP,
        GPIO_B_STEP, GPIO_C_STEP, GPIO_D_STEP
    };
    const size_t num_steps = sizeof(step_pins) / sizeof(step_pins[0]);

    bool toggle_ok = true;
    for (size_t i = 0; i < num_steps; i++) {
        esp_err_t ret = gpio_hal_set_level(step_pins[i], 1);
        if (ret != ESP_OK) toggle_ok = false;
        // Very brief pulse (GPIO toggle verification)
        gpio_hal_set_level(step_pins[i], 0);
    }

    if (toggle_ok) {
        result->passed = true;
        snprintf(result->message, sizeof(result->message),
                 "OK GPIO:ESTOP=%d STEPS=0", estop);
    } else {
        result->passed = false;
        snprintf(result->message, sizeof(result->message),
                 "ERROR GPIO:ESTOP=%d", estop);
    }

    ESP_LOGI(TAG, "GPIO test: %s", result->message);
    return ESP_OK;
}

esp_err_t test_all(test_result_t *result)
{
    if (result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    test_result_t i2c_result, sr_result, gpio_result;
    bool all_passed = true;

    test_i2c(&i2c_result);
    if (!i2c_result.passed) all_passed = false;

    test_sr(&sr_result);
    if (!sr_result.passed) all_passed = false;

    test_gpio(&gpio_result);
    if (!gpio_result.passed) all_passed = false;

    result->passed = all_passed;

    // Combine results with truncation to fit 128-byte buffer
    // Use precision specifier to limit each segment
    snprintf(result->message, sizeof(result->message),
             "%.38s | %.38s | %.38s",
             i2c_result.message, sr_result.message, gpio_result.message);

    return ESP_OK;
}
