/**
 * @file i2c_hal.c
 * @brief I2C HAL stub implementation
 * @author YaRobot Team
 * @date 2025
 */

#include "i2c_hal.h"
#include "esp_log.h"

static const char *TAG = "i2c_hal";

esp_err_t i2c_hal_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl,
                       uint32_t freq_hz)
{
    ESP_LOGI(TAG, "i2c_hal_init(port=%d, sda=%d, scl=%d, freq=%lu) stub called",
             port, sda, scl, (unsigned long)freq_hz);
    return ESP_OK;
}

esp_err_t i2c_hal_write(i2c_port_t port, uint8_t addr,
                        const uint8_t *data, size_t len)
{
    ESP_LOGI(TAG, "i2c_hal_write(port=%d, addr=0x%02X, len=%u) stub called",
             port, addr, (unsigned)len);
    (void)data;
    return ESP_OK;
}

esp_err_t i2c_hal_read(i2c_port_t port, uint8_t addr,
                       uint8_t *data, size_t len)
{
    ESP_LOGI(TAG, "i2c_hal_read(port=%d, addr=0x%02X, len=%u) stub called",
             port, addr, (unsigned)len);
    (void)data;
    return ESP_OK;
}

esp_err_t i2c_hal_write_read(i2c_port_t port, uint8_t addr,
                             const uint8_t *wr, size_t wr_len,
                             uint8_t *rd, size_t rd_len)
{
    ESP_LOGI(TAG, "i2c_hal_write_read(port=%d, addr=0x%02X, wr_len=%u, rd_len=%u) stub called",
             port, addr, (unsigned)wr_len, (unsigned)rd_len);
    (void)wr;
    (void)rd;
    return ESP_OK;
}
