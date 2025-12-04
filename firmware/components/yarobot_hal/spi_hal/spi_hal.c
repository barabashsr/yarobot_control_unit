/**
 * @file spi_hal.c
 * @brief SPI HAL stub implementation
 * @author YaRobot Team
 * @date 2025
 */

#include "spi_hal.h"
#include "esp_log.h"

static const char *TAG = "spi_hal";

esp_err_t spi_hal_init(spi_host_device_t host, gpio_num_t mosi,
                       gpio_num_t sclk, gpio_num_t cs)
{
    ESP_LOGI(TAG, "spi_hal_init(host=%d, mosi=%d, sclk=%d, cs=%d) stub called",
             host, mosi, sclk, cs);
    return ESP_OK;
}

esp_err_t spi_hal_transfer(spi_host_device_t host, const uint8_t *tx,
                           uint8_t *rx, size_t len)
{
    ESP_LOGI(TAG, "spi_hal_transfer(host=%d, len=%u) stub called",
             host, (unsigned)len);
    (void)tx;
    (void)rx;
    return ESP_OK;
}
