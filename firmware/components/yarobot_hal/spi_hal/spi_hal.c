/**
 * @file spi_hal.c
 * @brief SPI HAL implementation for shift register chain
 * @author YaRobot Team
 * @date 2025
 */

#include "spi_hal.h"
#include "esp_log.h"
#include "config_gpio.h"

static const char *TAG = "spi_hal";

/** @brief SPI clock frequency for shift registers (1 MHz) */
#define SPI_SR_FREQ_HZ  1000000

/** @brief SPI device handle for shift register */
static spi_device_handle_t s_sr_device = NULL;

/** @brief OE pin state tracking */
static gpio_num_t s_oe_pin = GPIO_NUM_NC;

/** @brief Initialization flag */
static bool s_initialized = false;

esp_err_t yarobot_spi_init(spi_host_device_t host, gpio_num_t mosi,
                           gpio_num_t sclk, gpio_num_t cs)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "SPI already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing SPI%d: MOSI=%d, SCLK=%d, CS=%d",
             host, mosi, sclk, cs);

    // Configure SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = mosi,
        .miso_io_num = -1,  // No MISO for shift registers
        .sclk_io_num = sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8,  // Max 5 bytes for 40 bits
    };

    esp_err_t ret = spi_bus_initialize(host, &bus_cfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure device for shift register chain
    // TPIC6B595N: Data latched on rising edge of RCLK (CS), shift on rising SRCLK
    spi_device_interface_config_t dev_cfg = {
        .mode = 0,  // CPOL=0, CPHA=0: data valid on rising edge
        .clock_speed_hz = SPI_SR_FREQ_HZ,
        .spics_io_num = cs,
        .queue_size = 1,
        .flags = 0,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    ret = spi_bus_add_device(host, &dev_cfg, &s_sr_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        spi_bus_free(host);
        return ret;
    }

    // Configure OE pin as output (active LOW)
    s_oe_pin = GPIO_SR_OE;
    gpio_config_t oe_cfg = {
        .pin_bit_mask = (1ULL << s_oe_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&oe_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure OE pin: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start with outputs disabled (OE HIGH)
    gpio_set_level(s_oe_pin, 1);

    s_initialized = true;
    ESP_LOGI(TAG, "SPI initialized successfully, OE pin=%d (disabled)", s_oe_pin);
    return ESP_OK;
}

bool spi_hal_is_initialized(void)
{
    return s_initialized;
}

esp_err_t spi_hal_transfer(spi_host_device_t host, const uint8_t *tx,
                           uint8_t *rx, size_t len)
{
    (void)host;  // We use the stored device handle

    if (!s_initialized || s_sr_device == NULL) {
        ESP_LOGE(TAG, "SPI not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t trans = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    return spi_device_transmit(s_sr_device, &trans);
}

esp_err_t spi_hal_sr_write(uint64_t data, size_t bits)
{
    if (!s_initialized || s_sr_device == NULL) {
        ESP_LOGE(TAG, "SPI not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (bits > 40) {
        ESP_LOGE(TAG, "Too many bits: %zu (max 40)", bits);
        return ESP_ERR_INVALID_ARG;
    }

    // Convert to bytes (MSB first for shift register chain)
    // Data shifts from first SR to last, so we send MSB first
    size_t bytes = (bits + 7) / 8;
    uint8_t tx_buf[5] = {0};

    // Pack data MSB first (highest byte first)
    for (size_t i = 0; i < bytes; i++) {
        tx_buf[i] = (data >> (8 * (bytes - 1 - i))) & 0xFF;
    }

    spi_transaction_t trans = {
        .length = bits,
        .tx_buffer = tx_buf,
        .rx_buffer = NULL,
    };

    esp_err_t ret = spi_device_transmit(s_sr_device, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SR write failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "SR: Wrote %zu bits (0x%llX)", bits, (unsigned long long)data);
    }

    return ret;
}

esp_err_t spi_hal_sr_set_oe(bool enable)
{
    if (s_oe_pin == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "OE pin not configured");
        return ESP_ERR_INVALID_STATE;
    }

    // OE is active LOW: enable=true -> level=0
    gpio_set_level(s_oe_pin, enable ? 0 : 1);
    ESP_LOGI(TAG, "SR outputs %s", enable ? "ENABLED" : "DISABLED");
    return ESP_OK;
}
