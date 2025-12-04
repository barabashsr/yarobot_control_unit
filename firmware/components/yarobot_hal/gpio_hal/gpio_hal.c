/**
 * @file gpio_hal.c
 * @brief GPIO HAL implementation
 * @author YaRobot Team
 * @date 2025
 */

#include "gpio_hal.h"
#include "esp_log.h"
#include "config_gpio.h"

static const char *TAG = "gpio_hal";

/** @brief Initialization flag */
static bool s_initialized = false;

/** @brief GPIO ISR service installed flag */
static bool s_isr_service_installed = false;

esp_err_t gpio_hal_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "GPIO HAL already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing GPIO HAL");

    // Configure STEP output pins
    gpio_config_t step_cfg = {
        .pin_bit_mask = (1ULL << GPIO_X_STEP) | (1ULL << GPIO_Y_STEP) |
                        (1ULL << GPIO_Z_STEP) | (1ULL << GPIO_A_STEP) |
                        (1ULL << GPIO_B_STEP) | (1ULL << GPIO_C_STEP) |
                        (1ULL << GPIO_D_STEP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&step_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure STEP pins: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize all STEP outputs to LOW
    gpio_set_level(GPIO_X_STEP, 0);
    gpio_set_level(GPIO_Y_STEP, 0);
    gpio_set_level(GPIO_Z_STEP, 0);
    gpio_set_level(GPIO_A_STEP, 0);
    gpio_set_level(GPIO_B_STEP, 0);
    gpio_set_level(GPIO_C_STEP, 0);
    gpio_set_level(GPIO_D_STEP, 0);

    // Configure E-STOP input with pull-up (active LOW)
    gpio_config_t estop_cfg = {
        .pin_bit_mask = (1ULL << GPIO_E_STOP),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ret = gpio_config(&estop_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure E-STOP pin: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure Z-Signal inputs (encoder index pulses)
    gpio_config_t zsig_cfg = {
        .pin_bit_mask = (1ULL << GPIO_X_Z_SIGNAL) | (1ULL << GPIO_Y_Z_SIGNAL) |
                        (1ULL << GPIO_Z_Z_SIGNAL) | (1ULL << GPIO_A_Z_SIGNAL) |
                        (1ULL << GPIO_B_Z_SIGNAL),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ret = gpio_config(&zsig_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Z-Signal pins: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure MCP23017 interrupt pins as inputs
    gpio_config_t mcp_int_cfg = {
        .pin_bit_mask = (1ULL << GPIO_MCP0_INTA) | (1ULL << GPIO_MCP0_INTB) |
                        (1ULL << GPIO_MCP1_INTA) | (1ULL << GPIO_MCP1_INTB),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ret = gpio_config(&mcp_int_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure MCP interrupt pins: %s", esp_err_to_name(ret));
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "GPIO HAL initialized successfully");
    return ESP_OK;
}

esp_err_t gpio_hal_set_direction(gpio_num_t pin, gpio_mode_t mode)
{
    esp_err_t ret = gpio_set_direction(pin, mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction for GPIO%d: %s", pin, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t gpio_hal_set_level(gpio_num_t pin, uint32_t level)
{
    esp_err_t ret = gpio_set_level(pin, level);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set level for GPIO%d: %s", pin, esp_err_to_name(ret));
    }
    return ret;
}

int gpio_hal_get_level(gpio_num_t pin)
{
    return gpio_get_level(pin);
}

esp_err_t gpio_hal_set_interrupt(gpio_num_t pin, gpio_int_type_t type,
                                  gpio_isr_t handler, void *arg)
{
    // Install GPIO ISR service if not already done
    if (!s_isr_service_installed) {
        esp_err_t ret = gpio_install_isr_service(0);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
            return ret;
        }
        s_isr_service_installed = true;
    }

    // Set interrupt type
    esp_err_t ret = gpio_set_intr_type(pin, type);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set interrupt type for GPIO%d: %s", pin, esp_err_to_name(ret));
        return ret;
    }

    // Add ISR handler
    ret = gpio_isr_handler_add(pin, handler, arg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler for GPIO%d: %s", pin, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Interrupt configured for GPIO%d, type=%d", pin, type);
    return ESP_OK;
}
