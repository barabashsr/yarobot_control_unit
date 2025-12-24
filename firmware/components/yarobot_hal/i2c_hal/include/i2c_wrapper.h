/**
 * @file i2c_wrapper.h
 * @brief I2C Wrapper API with automatic retry, bus recovery, and health tracking
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-11: I2C Communication Health
 *       AC1: Timeout detection (50ms via TIMING_I2C_TIMEOUT_MS)
 *       AC2: Auto-retry (TIMING_I2C_RETRY_COUNT = 3 attempts)
 *       AC3: Bus recovery (9 clock pulses, reinitialize)
 *
 * This wrapper sits on top of i2c_hal and provides:
 * - Automatic retry on transient failures
 * - Bus recovery after persistent failures
 * - Health tracking via i2c_health_monitor
 *
 * @see i2c_hal.h for low-level I2C operations
 * @see i2c_health_monitor.h for health tracking
 * @see config_timing.h for TIMING_I2C_* constants
 */

#ifndef I2C_WRAPPER_H
#define I2C_WRAPPER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup i2c_wrapper I2C Wrapper
 * @brief High-level I2C operations with retry and recovery
 * @{
 */

/**
 * @brief Initialize I2C wrapper
 *
 * Initializes the wrapper state. The underlying i2c_hal must be
 * initialized separately before using wrapper functions.
 *
 * @return esp_err_t
 *         - ESP_OK: Initialized successfully
 *         - ESP_ERR_INVALID_STATE: Already initialized
 */
esp_err_t i2c_wrapper_init(void);

/**
 * @brief Check if I2C wrapper is initialized
 *
 * @return true if initialized, false otherwise
 */
bool i2c_wrapper_is_initialized(void);

/**
 * @brief Read from I2C device register with retry and health tracking
 *
 * Performs a write-then-read transaction to read from a device register.
 * Automatically retries on failure up to TIMING_I2C_RETRY_COUNT times.
 * Triggers bus recovery if all retries fail.
 * Records success/failure with i2c_health_monitor.
 *
 * @param[in] port I2C port
 * @param[in] addr 7-bit device address
 * @param[in] reg Register address to read from
 * @param[out] data Buffer to receive read data
 * @param[in] len Number of bytes to read
 *
 * @return esp_err_t
 *         - ESP_OK: Read successful
 *         - ESP_ERR_TIMEOUT: Device not responding after retries
 *         - ESP_ERR_INVALID_STATE: I2C not initialized
 *         - ESP_ERR_INVALID_ARG: Invalid parameters
 */
esp_err_t i2c_wrapper_read(i2c_port_t port, uint8_t addr, uint8_t reg,
                           uint8_t *data, size_t len);

/**
 * @brief Write to I2C device register with retry and health tracking
 *
 * Writes data to a device register (register address + data).
 * Automatically retries on failure up to TIMING_I2C_RETRY_COUNT times.
 * Triggers bus recovery if all retries fail.
 * Records success/failure with i2c_health_monitor.
 *
 * @param[in] port I2C port
 * @param[in] addr 7-bit device address
 * @param[in] reg Register address to write to
 * @param[in] data Data to write
 * @param[in] len Number of data bytes to write
 *
 * @return esp_err_t
 *         - ESP_OK: Write successful
 *         - ESP_ERR_TIMEOUT: Device not responding after retries
 *         - ESP_ERR_INVALID_STATE: I2C not initialized
 *         - ESP_ERR_INVALID_ARG: Invalid parameters
 */
esp_err_t i2c_wrapper_write(i2c_port_t port, uint8_t addr, uint8_t reg,
                            const uint8_t *data, size_t len);

/**
 * @brief Raw I2C read with retry (no register address)
 *
 * Reads directly from device without sending register address first.
 * Useful for devices that auto-increment or don't use register addressing.
 *
 * @param[in] port I2C port
 * @param[in] addr 7-bit device address
 * @param[out] data Buffer to receive read data
 * @param[in] len Number of bytes to read
 *
 * @return esp_err_t (same as i2c_wrapper_read)
 */
esp_err_t i2c_wrapper_read_raw(i2c_port_t port, uint8_t addr,
                               uint8_t *data, size_t len);

/**
 * @brief Raw I2C write with retry (no register address)
 *
 * Writes directly to device without register addressing.
 *
 * @param[in] port I2C port
 * @param[in] addr 7-bit device address
 * @param[in] data Data to write
 * @param[in] len Number of bytes to write
 *
 * @return esp_err_t (same as i2c_wrapper_write)
 */
esp_err_t i2c_wrapper_write_raw(i2c_port_t port, uint8_t addr,
                                const uint8_t *data, size_t len);

/**
 * @brief Attempt bus recovery
 *
 * Performs I2C bus recovery procedure:
 * 1. Set SDA to input mode
 * 2. Toggle SCL 9 times (clock out stuck slave)
 * 3. Generate STOP condition
 * 4. Reinitialize I2C bus
 *
 * Called automatically after TIMING_I2C_RETRY_COUNT consecutive failures.
 * Can also be called manually if bus is known to be stuck.
 *
 * @param[in] port I2C port to recover
 *
 * @return esp_err_t
 *         - ESP_OK: Recovery attempted (bus may or may not be restored)
 *         - ESP_ERR_INVALID_ARG: Invalid port
 *         - ESP_ERR_INVALID_STATE: I2C not initialized
 */
esp_err_t i2c_wrapper_recover_bus(i2c_port_t port);

/**
 * @brief Probe device to check if it's responding
 *
 * Sends a quick probe to check device presence.
 * Does NOT record health (use for initial scan only).
 *
 * @param[in] port I2C port
 * @param[in] addr 7-bit device address
 *
 * @return esp_err_t
 *         - ESP_OK: Device responding
 *         - ESP_ERR_TIMEOUT: Device not responding
 */
esp_err_t i2c_wrapper_probe(i2c_port_t port, uint8_t addr);

/**
 * @brief Get retry count statistics
 *
 * Returns the number of retries that have occurred since initialization.
 * Useful for diagnostics.
 *
 * @return Total retry count
 */
uint32_t i2c_wrapper_get_retry_count(void);

/**
 * @brief Get recovery attempt count
 *
 * Returns the number of bus recovery attempts since initialization.
 *
 * @return Total recovery attempts
 */
uint32_t i2c_wrapper_get_recovery_count(void);

/** @} */ // end i2c_wrapper

#ifdef __cplusplus
}
#endif

#endif // I2C_WRAPPER_H
