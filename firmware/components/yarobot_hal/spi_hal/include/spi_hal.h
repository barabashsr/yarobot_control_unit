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
esp_err_t spi_hal_init(spi_host_device_t host, gpio_num_t mosi,
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
 *
 * @note This is a stub implementation - returns ESP_OK without side effects
 */
esp_err_t spi_hal_transfer(spi_host_device_t host, const uint8_t *tx,
                           uint8_t *rx, size_t len);

/** @} */ // end spi_hal

#ifdef __cplusplus
}
#endif

#endif // SPI_HAL_H
