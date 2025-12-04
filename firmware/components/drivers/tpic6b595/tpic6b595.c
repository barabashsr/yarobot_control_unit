/**
 * @file tpic6b595.c
 * @brief TPIC6B595N Shift Register Driver Implementation
 * @author YaRobot Team
 * @date 2025
 */

#include "tpic6b595.h"
#include "config_sr.h"
#include "config_gpio.h"
#include "spi_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/spi_struct.h"

static const char *TAG = "tpic6b595";

/** @brief Shadow register for shift register state (40 bits used) */
static volatile uint64_t s_shadow_register = SR_SAFE_STATE;

/** @brief Mutex for thread-safe access */
static SemaphoreHandle_t s_mutex = NULL;

/** @brief Initialization flag */
static bool s_initialized = false;

/*******************************************************************************
 * Private Helper Functions
 ******************************************************************************/

/**
 * @brief Set a bit in the shadow register
 */
static inline void set_bit(uint8_t position, bool value)
{
    if (value) {
        s_shadow_register |= (1ULL << position);
    } else {
        s_shadow_register &= ~(1ULL << position);
    }
}

/**
 * @brief Get a bit from the shadow register
 */
static inline bool get_bit(uint8_t position)
{
    return (s_shadow_register >> position) & 1;
}

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

esp_err_t sr_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing TPIC6B595 driver");

    // Initialize SPI HAL if not already done
    if (!spi_hal_is_initialized()) {
        esp_err_t ret = yarobot_spi_init(SPI2_HOST, GPIO_SR_MOSI, GPIO_SR_SCLK, GPIO_SR_CS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI HAL init failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    // Create mutex for thread safety
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize shadow register to safe state
    s_shadow_register = SR_SAFE_STATE;

    // Latch safe state to hardware
    esp_err_t ret = spi_hal_sr_write(s_shadow_register, SR_BITS_TOTAL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to latch safe state: %s", esp_err_to_name(ret));
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return ret;
    }

    // Enable outputs (OE LOW)
    ret = spi_hal_sr_set_oe(true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable outputs: %s", esp_err_to_name(ret));
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "TPIC6B595 driver initialized, 40-bit chain ready");
    return ESP_OK;
}

bool sr_is_initialized(void)
{
    return s_initialized;
}

esp_err_t sr_set_direction(uint8_t axis, bool forward)
{
    if (axis >= SR_NUM_AXES) {
        ESP_LOGE(TAG, "Invalid axis: %d", axis);
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    set_bit(SR_DIR_BIT(axis), forward);

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t sr_set_enable(uint8_t axis, bool enable)
{
    if (axis >= SR_NUM_AXES) {
        ESP_LOGE(TAG, "Invalid axis: %d", axis);
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    set_bit(SR_EN_BIT(axis), enable);

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t sr_set_brake(uint8_t axis, bool release)
{
    // E-axis (7) has no brake
    if (axis > 6) {
        ESP_LOGE(TAG, "Invalid axis for brake: %d (E-axis has no brake)", axis);
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    set_bit(SR_BRAKE_BIT(axis), release);

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t sr_set_alarm_clear(uint8_t axis, bool active)
{
    // E-axis (7) has no alarm clear
    if (axis > 6) {
        ESP_LOGE(TAG, "Invalid axis for alarm clear: %d (E-axis has no alarm)", axis);
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    set_bit(SR_ALARM_CLR_BIT(axis), active);

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t sr_set_gp_output(uint8_t pin, bool level)
{
    if (pin >= SR_NUM_GP_OUTPUTS) {
        ESP_LOGE(TAG, "Invalid GP pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    set_bit(SR_GP_OUT_0 + pin, level);

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t sr_update(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Write shadow register to hardware via SPI HAL
    // The HAL handles the SPI transfer and latch pulse
    esp_err_t ret = spi_hal_sr_write(s_shadow_register, SR_BITS_TOTAL);

    xSemaphoreGive(s_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI write failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

uint64_t sr_get_state(void)
{
    // Read is atomic for uint64_t on ESP32-S3 with proper alignment
    return s_shadow_register;
}

void sr_emergency_disable_all(void)
{
    // ISR-SAFE: No mutex, no blocking calls
    // Must complete within 100us

    // Step 1: Immediately tristate outputs (OE HIGH)
    gpio_set_level(GPIO_SR_OE, 1);

    // Step 2: Clear shadow register to safe state
    s_shadow_register = SR_SAFE_STATE;

    // Step 3: Direct SPI write without HAL (for speed and ISR safety)
    // Use polling mode SPI for minimal latency
    // Access SPI2 hardware directly for ISR safety
    // This bypasses the driver mutex but is acceptable for emergency
    spi_dev_t *hw = &GPSPI2;

    // Wait for any ongoing transaction (should be quick)
    while (hw->cmd.usr) {
        // Spin
    }

    // Configure for 40-bit transfer
    hw->user.usr_mosi = 1;
    hw->user.usr_miso = 0;
    hw->ms_dlen.ms_data_bitlen = 39;  // 40 - 1

    // Load data (MSB first)
    hw->data_buf[0] = 0;
    hw->data_buf[1] = 0;

    // Start transfer
    hw->cmd.usr = 1;

    // Wait for completion (polling, no RTOS)
    while (hw->cmd.usr) {
        // Spin
    }

    // Pulse CS/LATCH to latch data (CS is active low, pulse high-low)
    // Note: The SPI driver already toggles CS, but we pulse explicitly for latch
    gpio_set_level(GPIO_SR_CS, 0);
    gpio_set_level(GPIO_SR_CS, 1);
    gpio_set_level(GPIO_SR_CS, 0);

    // OE remains HIGH (outputs tristated) for safety
    // Recovery requires explicit sr_enable_outputs() call
}

esp_err_t sr_enable_outputs(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Set OE LOW to enable outputs
    return spi_hal_sr_set_oe(true);
}
