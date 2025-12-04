/**
 * @file i2c_hal.h
 * @brief I2C Hardware Abstraction Layer for YaRobot Control Unit
 * @author YaRobot Team
 * @date 2025
 *
 * @note Provides a hardware abstraction for I2C master operations.
 *       Stub implementation returns ESP_OK without side effects.
 */

#ifndef I2C_HAL_H
#define I2C_HAL_H

#include "esp_err.h"
#include "driver/i2c_types.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup i2c_hal I2C Hardware Abstraction Layer
 * @brief Hardware abstraction for I2C master operations
 * @{
 */

/**
 * @brief Initialize I2C master bus
 *
 * @param[in] port I2C port number (I2C_NUM_0 or I2C_NUM_1)
 * @param[in] sda GPIO pin for SDA line
 * @param[in] scl GPIO pin for SCL line
 * @param[in] freq_hz Bus clock frequency in Hz
 *
 * @return esp_err_t ESP_OK on success
 *
 * @note This is a stub implementation - returns ESP_OK without side effects
 */
esp_err_t i2c_hal_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl,
                       uint32_t freq_hz);

/**
 * @brief Write data to I2C device
 *
 * @param[in] port I2C port number
 * @param[in] addr 7-bit device address
 * @param[in] data Pointer to data buffer
 * @param[in] len Number of bytes to write
 *
 * @return esp_err_t ESP_OK on success
 *
 * @note This is a stub implementation - returns ESP_OK without side effects
 */
esp_err_t i2c_hal_write(i2c_port_t port, uint8_t addr,
                        const uint8_t *data, size_t len);

/**
 * @brief Read data from I2C device
 *
 * @param[in] port I2C port number
 * @param[in] addr 7-bit device address
 * @param[out] data Pointer to buffer for received data
 * @param[in] len Number of bytes to read
 *
 * @return esp_err_t ESP_OK on success
 *
 * @note This is a stub implementation - returns ESP_OK without side effects
 */
esp_err_t i2c_hal_read(i2c_port_t port, uint8_t addr,
                       uint8_t *data, size_t len);

/**
 * @brief Write then read data from I2C device (combined transaction)
 *
 * @param[in] port I2C port number
 * @param[in] addr 7-bit device address
 * @param[in] wr Pointer to write data buffer
 * @param[in] wr_len Number of bytes to write
 * @param[out] rd Pointer to buffer for received data
 * @param[in] rd_len Number of bytes to read
 *
 * @return esp_err_t ESP_OK on success
 *
 * @note This is a stub implementation - returns ESP_OK without side effects
 */
esp_err_t i2c_hal_write_read(i2c_port_t port, uint8_t addr,
                             const uint8_t *wr, size_t wr_len,
                             uint8_t *rd, size_t rd_len);

/** @} */ // end i2c_hal

#ifdef __cplusplus
}
#endif

#endif // I2C_HAL_H
