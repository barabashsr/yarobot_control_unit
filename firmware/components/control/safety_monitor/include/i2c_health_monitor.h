/**
 * @file i2c_health_monitor.h
 * @brief I2C Health Monitor API - Tracks I2C device health and bus status
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-11: I2C Communication Health
 *       - Tracks transaction counts and failure counts per device
 *       - Detects device offline after I2C_FAILURE_THRESHOLD consecutive failures
 *       - Publishes health degradation and recovery events
 *       - Thread-safe access via FreeRTOS mutex
 *
 * @see config_i2c.h for I2C_ADDR_MCP23017_0, I2C_ADDR_MCP23017_1
 * @see config_limits.h for I2C_FAILURE_THRESHOLD, I2C_RECOVERY_THRESHOLD
 * @see config_commands.h for EVT_I2C_RECOVERED, EVT_LIMITS_DEGRADED, EVT_FEEDBACK_DEGRADED
 */

#ifndef I2C_HEALTH_MONITOR_H
#define I2C_HEALTH_MONITOR_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup i2c_health I2C Health Monitor
 * @brief I2C communication health tracking and device status management
 * @{
 */

/**
 * @brief Maximum number of tracked I2C devices
 */
#define I2C_MAX_DEVICES     8

/**
 * @brief Maximum device name length
 */
#define I2C_DEVICE_NAME_LEN 16

/**
 * @brief I2C bus health status
 */
typedef enum {
    I2C_HEALTH_OK = 0,      /**< Bus healthy, all transactions succeeding */
    I2C_HEALTH_WARN,        /**< Bus has had failures but is operational */
    I2C_HEALTH_ERROR        /**< Bus has critical errors or device offline */
} I2CHealthStatus;

/**
 * @brief I2C bus health summary
 */
typedef struct {
    I2CHealthStatus status;     /**< Overall health status */
    uint32_t success_count;     /**< Total successful transactions */
    uint32_t failure_count;     /**< Total failed transactions */
    uint8_t online_device_count; /**< Number of online devices on this bus */
    uint8_t total_device_count;  /**< Total tracked devices on this bus */
} I2CBusHealth;

/**
 * @brief I2C device health state
 */
typedef struct {
    uint8_t address;                    /**< Device I2C address */
    i2c_port_t port;                    /**< I2C port (bus) */
    char name[I2C_DEVICE_NAME_LEN];     /**< Device name (e.g., "MCP23017") */
    bool is_online;                     /**< Device online status */
    uint32_t transaction_count;         /**< Successful transaction count */
    uint32_t failure_count;             /**< Total failure count */
    uint32_t consecutive_failures;      /**< Consecutive failures (clears on success) */
    uint32_t consecutive_successes;     /**< Consecutive successes (clears on failure) */
} I2CDeviceHealth;

/**
 * @defgroup i2c_health_api I2C Health Monitor API
 * @brief Initialization and health management functions
 * @{
 */

/**
 * @brief Initialize I2C health monitor
 *
 * Performs the following:
 * - Creates mutex for thread-safe access
 * - Registers known I2C devices (MCP23017 x2)
 * - Marks all devices as online initially
 *
 * @return esp_err_t
 *         - ESP_OK: Initialized successfully
 *         - ESP_ERR_INVALID_STATE: Already initialized
 *         - ESP_ERR_NO_MEM: Failed to create mutex
 */
esp_err_t i2c_health_init(void);

/**
 * @brief Check if I2C health monitor is initialized
 *
 * @return true if initialized, false otherwise
 */
bool i2c_health_is_initialized(void);

/**
 * @brief Deinitialize I2C health monitor
 *
 * Resets all state and frees mutex. Safe to call if not initialized.
 *
 * @return esp_err_t
 *         - ESP_OK: Deinitialized successfully
 */
esp_err_t i2c_health_deinit(void);

/** @} */ // end i2c_health_api

/**
 * @defgroup i2c_health_tracking Health Tracking Functions
 * @brief Functions for recording transaction results (AC1, AC2, AC4)
 * @{
 */

/**
 * @brief Record successful I2C transaction
 *
 * Increments success counter, clears consecutive failure count.
 * If device was offline and consecutive successes reach I2C_RECOVERY_THRESHOLD,
 * device is marked online and recovery event published.
 *
 * @param[in] port I2C port (bus)
 * @param[in] addr Device I2C address
 *
 * @return esp_err_t
 *         - ESP_OK: Success recorded
 *         - ESP_ERR_NOT_FOUND: Device not tracked
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t i2c_health_record_success(i2c_port_t port, uint8_t addr);

/**
 * @brief Record failed I2C transaction
 *
 * Increments failure counter and consecutive failure count.
 * If consecutive failures reach I2C_FAILURE_THRESHOLD:
 * - Device marked offline
 * - Degradation event published
 * - Error count incremented via error_manager
 *
 * @param[in] port I2C port (bus)
 * @param[in] addr Device I2C address
 *
 * @return esp_err_t
 *         - ESP_OK: Failure recorded
 *         - ESP_ERR_NOT_FOUND: Device not tracked
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t i2c_health_record_failure(i2c_port_t port, uint8_t addr);

/** @} */ // end i2c_health_tracking

/**
 * @defgroup i2c_health_status Status Query Functions
 * @brief Functions for querying health status (AC5, AC6, AC7)
 * @{
 */

/**
 * @brief Get bus health summary
 *
 * Returns aggregate health status for an I2C port.
 *
 * @param[in] port I2C port (bus)
 * @param[out] health Pointer to receive bus health summary
 *
 * @return esp_err_t
 *         - ESP_OK: Health retrieved
 *         - ESP_ERR_INVALID_ARG: health is NULL
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t i2c_health_get_bus_status(i2c_port_t port, I2CBusHealth *health);

/**
 * @brief Check if device is online
 *
 * @param[in] addr Device I2C address
 *
 * @return true if device is online (or not tracked), false if offline
 */
bool i2c_health_is_device_online(uint8_t addr);

/**
 * @brief Get device health details
 *
 * @param[in] addr Device I2C address
 * @param[out] health Pointer to receive device health
 *
 * @return esp_err_t
 *         - ESP_OK: Health retrieved
 *         - ESP_ERR_NOT_FOUND: Device not tracked
 *         - ESP_ERR_INVALID_ARG: health is NULL
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t i2c_health_get_device_status(uint8_t addr, I2CDeviceHealth *health);

/**
 * @brief Get all tracked devices
 *
 * @param[out] devices Array to receive device health info
 * @param[in] max_devices Maximum entries in array
 * @param[out] count Number of devices returned
 *
 * @return esp_err_t
 *         - ESP_OK: Devices retrieved
 *         - ESP_ERR_INVALID_ARG: devices or count is NULL
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t i2c_health_get_all_devices(I2CDeviceHealth *devices, size_t max_devices, size_t *count);

/**
 * @brief Get device name by address
 *
 * @param[in] addr Device I2C address
 *
 * @return Device name string, or "UNKNOWN" if not tracked
 */
const char* i2c_health_get_device_name(uint8_t addr);

/** @} */ // end i2c_health_status

/**
 * @defgroup i2c_health_device Device Registration
 * @brief Functions for registering devices
 * @{
 */

/**
 * @brief Register a device for health tracking
 *
 * @param[in] port I2C port (bus)
 * @param[in] addr Device I2C address
 * @param[in] name Device name (e.g., "MCP23017")
 *
 * @return esp_err_t
 *         - ESP_OK: Device registered
 *         - ESP_ERR_NO_MEM: Max devices reached
 *         - ESP_ERR_INVALID_ARG: name is NULL or addr is 0
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t i2c_health_register_device(i2c_port_t port, uint8_t addr, const char *name);

/** @} */ // end i2c_health_device

/**
 * @defgroup i2c_health_test Test Functions
 * @brief Functions for testing
 * @{
 */

/**
 * @brief Inject device offline state for testing
 *
 * @param[in] addr Device address
 * @param[in] offline true to mark offline, false to mark online
 */
void i2c_health_inject_offline(uint8_t addr, bool offline);

/** @} */ // end i2c_health_test

/** @} */ // end i2c_health

#ifdef __cplusplus
}
#endif

#endif // I2C_HEALTH_MONITOR_H
