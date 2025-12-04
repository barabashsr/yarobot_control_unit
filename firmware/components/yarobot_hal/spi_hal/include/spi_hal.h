/**
 * @file spi_hal.h
 * @brief SPI Hardware Abstraction Layer for YaRobot Control Unit
 * @author YaRobot Team
 * @date 2025
 *
 * @note Provides a hardware abstraction for SPI master operations.
 *       Used primarily for shift register communication.
 *       Stub implementation returns ESP_OK without side effects.
 */

#ifndef SPI_HAL_H
#define SPI_HAL_H

#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup spi_hal SPI Hardware Abstraction Layer
 * @brief Hardware abstraction for SPI master operations
 * @{
 */

/**
 * @brief Initialize SPI master bus
 *
 * @param[in] host SPI host device (SPI2_HOST or SPI3_HOST)
 * @param[in] mosi GPIO pin for MOSI line
 * @param[in] sclk GPIO pin for SCLK line
 * @param[in] cs GPIO pin for chip select line
 *
 * @return esp_err_t ESP_OK on success
 *
 * @note This is a stub implementation - returns ESP_OK without side effects
 */
esp_err_t yarobot_spi_init(spi_host_device_t host, gpio_num_t mosi,
                           gpio_num_t sclk, gpio_num_t cs);

/**
 * @brief Transfer data over SPI (full-duplex)
 *
 * @param[in] host SPI host device
 * @param[in] tx Pointer to transmit buffer (can be NULL for read-only)
 * @param[out] rx Pointer to receive buffer (can be NULL for write-only)
 * @param[in] len Number of bytes to transfer
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_hal_transfer(spi_host_device_t host, const uint8_t *tx,
                           uint8_t *rx, size_t len);

/**
 * @brief Write data to shift register chain
 *
 * Shifts data out MSB first and latches on completion.
 *
 * @param[in] data 40-bit data to shift out (only lower 40 bits used)
 * @param[in] bits Number of bits to shift (typically 24 for test, 40 for full chain)
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_hal_sr_write(uint64_t data, size_t bits);

/**
 * @brief Set shift register output enable state
 *
 * @param[in] enable true to enable outputs (OE LOW), false to disable (OE HIGH)
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_hal_sr_set_oe(bool enable);

/**
 * @brief Check if SPI is initialized
 *
 * @return true if initialized, false otherwise
 */
bool spi_hal_is_initialized(void);

/** @} */ // end spi_hal

#ifdef __cplusplus
}
#endif

#endif // SPI_HAL_H
