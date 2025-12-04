/**
 * @file gpio_hal.c
 * @brief GPIO HAL stub implementation
 * @author YaRobot Team
 * @date 2025
 */

#include "gpio_hal.h"
#include "esp_log.h"

static const char *TAG = "gpio_hal";

esp_err_t gpio_hal_init(void)
{
    ESP_LOGI(TAG, "gpio_hal_init() stub called");
    return ESP_OK;
}

esp_err_t gpio_hal_set_direction(gpio_num_t pin, gpio_mode_t mode)
{
    ESP_LOGI(TAG, "gpio_hal_set_direction(pin=%d, mode=%d) stub called", pin, mode);
    return ESP_OK;
}

esp_err_t gpio_hal_set_level(gpio_num_t pin, uint32_t level)
{
    ESP_LOGI(TAG, "gpio_hal_set_level(pin=%d, level=%lu) stub called", pin, (unsigned long)level);
    return ESP_OK;
}

int gpio_hal_get_level(gpio_num_t pin)
{
    ESP_LOGI(TAG, "gpio_hal_get_level(pin=%d) stub called", pin);
    return 0;
}

esp_err_t gpio_hal_set_interrupt(gpio_num_t pin, gpio_int_type_t type,
                                  gpio_isr_t handler, void *arg)
{
    ESP_LOGI(TAG, "gpio_hal_set_interrupt(pin=%d, type=%d) stub called", pin, type);
    (void)handler;
    (void)arg;
    return ESP_OK;
}
