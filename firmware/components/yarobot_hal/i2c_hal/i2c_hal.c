/**
 * @file i2c_hal.c
 * @brief I2C HAL implementation using ESP-IDF 5.4 I2C master driver
 * @author YaRobot Team
 * @date 2025
 */

#include "i2c_hal.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "i2c_hal";

/** @brief Maximum number of I2C ports */
#define I2C_HAL_MAX_PORTS 2

/** @brief I2C probe timeout in milliseconds */
#define I2C_PROBE_TIMEOUT_MS 50

/** @brief I2C transaction timeout in milliseconds */
#define I2C_XFER_TIMEOUT_MS 100

/** @brief Bus handle storage for each port */
static i2c_master_bus_handle_t s_bus_handles[I2C_HAL_MAX_PORTS] = {NULL, NULL};

/** @brief Bus frequencies for device configuration */
static uint32_t s_bus_freq[I2C_HAL_MAX_PORTS] = {0, 0};

esp_err_t i2c_hal_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl,
                       uint32_t freq_hz)
{
    if (port >= I2C_HAL_MAX_PORTS) {
        ESP_LOGE(TAG, "Invalid I2C port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }

    if (s_bus_handles[port] != NULL) {
        ESP_LOGW(TAG, "I2C%d already initialized", port);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing I2C%d: SDA=%d, SCL=%d, freq=%lu Hz",
             port, sda, scl, (unsigned long)freq_hz);

    i2c_master_bus_config_t bus_config = {
        .i2c_port = port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &s_bus_handles[port]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C%d master bus: %s", port, esp_err_to_name(ret));
        return ret;
    }

    s_bus_freq[port] = freq_hz;
    ESP_LOGI(TAG, "I2C%d initialized successfully", port);
    return ESP_OK;
}

bool i2c_hal_is_initialized(i2c_port_t port)
{
    if (port >= I2C_HAL_MAX_PORTS) {
        return false;
    }
    return s_bus_handles[port] != NULL;
}

esp_err_t i2c_hal_write(i2c_port_t port, uint8_t addr,
                        const uint8_t *data, size_t len)
{
    if (port >= I2C_HAL_MAX_PORTS || s_bus_handles[port] == NULL) {
        ESP_LOGE(TAG, "I2C%d not initialized", port);
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = s_bus_freq[port],
    };

    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(s_bus_handles[port], &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device 0x%02X: %s", addr, esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_master_transmit(dev_handle, data, len, I2C_XFER_TIMEOUT_MS);
    i2c_master_bus_rm_device(dev_handle);

    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "I2C%d write to 0x%02X failed: %s", port, addr, esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t i2c_hal_read(i2c_port_t port, uint8_t addr,
                       uint8_t *data, size_t len)
{
    if (port >= I2C_HAL_MAX_PORTS || s_bus_handles[port] == NULL) {
        ESP_LOGE(TAG, "I2C%d not initialized", port);
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = s_bus_freq[port],
    };

    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(s_bus_handles[port], &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device 0x%02X: %s", addr, esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_master_receive(dev_handle, data, len, I2C_XFER_TIMEOUT_MS);
    i2c_master_bus_rm_device(dev_handle);

    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "I2C%d read from 0x%02X failed: %s", port, addr, esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t i2c_hal_write_read(i2c_port_t port, uint8_t addr,
                             const uint8_t *wr, size_t wr_len,
                             uint8_t *rd, size_t rd_len)
{
    if (port >= I2C_HAL_MAX_PORTS || s_bus_handles[port] == NULL) {
        ESP_LOGE(TAG, "I2C%d not initialized", port);
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = s_bus_freq[port],
    };

    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(s_bus_handles[port], &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device 0x%02X: %s", addr, esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_master_transmit_receive(dev_handle, wr, wr_len, rd, rd_len, I2C_XFER_TIMEOUT_MS);
    i2c_master_bus_rm_device(dev_handle);

    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "I2C%d write_read 0x%02X failed: %s", port, addr, esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t i2c_hal_scan_bus(i2c_port_t port, uint8_t *found_addrs,
                           size_t max_addrs, size_t *count)
{
    if (port >= I2C_HAL_MAX_PORTS || s_bus_handles[port] == NULL) {
        ESP_LOGE(TAG, "I2C%d not initialized", port);
        return ESP_ERR_INVALID_STATE;
    }

    if (found_addrs == NULL || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *count = 0;
    ESP_LOGI(TAG, "Scanning I2C%d bus...", port);

    // Scan valid 7-bit address range (0x08 to 0x77)
    for (uint8_t addr = 0x08; addr < 0x78 && *count < max_addrs; addr++) {
        esp_err_t ret = i2c_master_probe(s_bus_handles[port], addr, I2C_PROBE_TIMEOUT_MS);
        if (ret == ESP_OK) {
            found_addrs[(*count)++] = addr;
            ESP_LOGI(TAG, "I2C%d: Found device at 0x%02X", port, addr);
        }
    }

    ESP_LOGI(TAG, "I2C%d scan complete: %zu device(s) found", port, *count);
    return ESP_OK;
}
