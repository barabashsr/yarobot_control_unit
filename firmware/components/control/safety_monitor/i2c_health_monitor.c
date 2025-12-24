/**
 * @file i2c_health_monitor.c
 * @brief I2C Health Monitor implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-11: I2C Communication Health
 *       AC1: Timeout detection (tracked via failure recording)
 *       AC2: Auto-retry tracking (wrapper calls record_success/failure)
 *       AC3: Bus recovery tracking (wrapper handles, this tracks results)
 *       AC4: Health degradation events published
 *       AC7: Device offline detection
 *       AC8: Graceful degradation events
 */

#include "i2c_health_monitor.h"
#include "config_i2c.h"
#include "config_limits.h"
#include "config_commands.h"
#include "error_manager.h"
#include "event_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"

#include <string.h>

static const char* TAG = "i2c_health";

/**
 * @brief I2C health monitor state
 */
static struct {
    bool initialized;                           /**< Initialization flag */
    SemaphoreHandle_t mutex;                    /**< Protects state access */

    // Tracked devices
    I2CDeviceHealth devices[I2C_MAX_DEVICES];   /**< Device health records */
    size_t device_count;                        /**< Number of registered devices */

} s_i2c_health = {
    .initialized = false,
    .mutex = NULL,
    .device_count = 0,
};

// ----------------------------------------------------------------------------
// Private Helper Functions
// ----------------------------------------------------------------------------

/**
 * @brief Take mutex with timeout
 */
static inline bool take_mutex(void)
{
    if (s_i2c_health.mutex == NULL) {
        return false;
    }
    return xSemaphoreTake(s_i2c_health.mutex, pdMS_TO_TICKS(100)) == pdTRUE;
}

/**
 * @brief Give mutex
 */
static inline void give_mutex(void)
{
    if (s_i2c_health.mutex != NULL) {
        xSemaphoreGive(s_i2c_health.mutex);
    }
}

/**
 * @brief Find device by address
 *
 * @param[in] addr Device address
 * @return Pointer to device record, or NULL if not found
 */
static I2CDeviceHealth* find_device(uint8_t addr)
{
    for (size_t i = 0; i < s_i2c_health.device_count; i++) {
        if (s_i2c_health.devices[i].address == addr) {
            return &s_i2c_health.devices[i];
        }
    }
    return NULL;
}

/**
 * @brief Publish degradation event based on device address
 *
 * Uses EVTTYPE_ERROR with appropriate error codes for I2C device offline events.
 * The event system will format this as "EVENT ERROR SYSTEM E022" for device offline.
 */
static void publish_degradation_event(uint8_t addr)
{
    if (addr == I2C_ADDR_MCP23017_0) {
        // MCP23017 #0 handles limit switches
        ESP_LOGW(TAG, "MCP23017 #0 (0x%02X) offline - limits degraded", addr);
    } else if (addr == I2C_ADDR_MCP23017_1) {
        // MCP23017 #1 handles InPos/ALARM
        ESP_LOGW(TAG, "MCP23017 #1 (0x%02X) offline - feedback degraded", addr);
    }

    // Use EVTTYPE_ERROR for device offline notification
    Event evt = {
        .type = EVTTYPE_ERROR,
        .axis = 0xFF,  // System-wide
        .timestamp = esp_timer_get_time()
    };
    event_publish(&evt);

    ESP_LOGE(TAG, "Device 0x%02X marked OFFLINE after %d consecutive failures",
             addr, I2C_FAILURE_THRESHOLD);
}

/**
 * @brief Log recovery event
 *
 * Recovery is logged but not published as a separate event type.
 * The health status can be queried via CMD_I2C command.
 */
static void publish_recovery_event(uint8_t addr)
{
    ESP_LOGI(TAG, "Device 0x%02X recovered after %d consecutive successes",
             addr, I2C_RECOVERY_THRESHOLD);
    // Note: No dedicated recovery event type exists in the system.
    // Recovery state is queryable via i2c_health_is_device_online() and CMD_I2C.
}

// ----------------------------------------------------------------------------
// Initialization API
// ----------------------------------------------------------------------------

esp_err_t i2c_health_init(void)
{
    if (s_i2c_health.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Create mutex
    s_i2c_health.mutex = xSemaphoreCreateMutex();
    if (s_i2c_health.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Clear device records
    memset(s_i2c_health.devices, 0, sizeof(s_i2c_health.devices));
    s_i2c_health.device_count = 0;

    s_i2c_health.initialized = true;

    // Register known devices
    i2c_health_register_device(I2C_PORT, I2C_ADDR_MCP23017_0, "MCP23017");
    i2c_health_register_device(I2C_PORT, I2C_ADDR_MCP23017_1, "MCP23017");

    ESP_LOGI(TAG, "I2C health monitor initialized - %zu devices tracked",
             s_i2c_health.device_count);

    return ESP_OK;
}

bool i2c_health_is_initialized(void)
{
    return s_i2c_health.initialized;
}

esp_err_t i2c_health_deinit(void)
{
    if (!s_i2c_health.initialized) {
        return ESP_OK;
    }

    // Delete mutex
    if (s_i2c_health.mutex != NULL) {
        vSemaphoreDelete(s_i2c_health.mutex);
        s_i2c_health.mutex = NULL;
    }

    // Clear state
    memset(s_i2c_health.devices, 0, sizeof(s_i2c_health.devices));
    s_i2c_health.device_count = 0;
    s_i2c_health.initialized = false;

    ESP_LOGI(TAG, "I2C health monitor deinitialized");
    return ESP_OK;
}

// ----------------------------------------------------------------------------
// Health Tracking Functions
// ----------------------------------------------------------------------------

esp_err_t i2c_health_record_success(i2c_port_t port, uint8_t addr)
{
    (void)port;  // Currently not used for lookup, just addr

    if (!s_i2c_health.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!take_mutex()) {
        return ESP_ERR_TIMEOUT;
    }

    I2CDeviceHealth* dev = find_device(addr);
    if (dev == NULL) {
        give_mutex();
        return ESP_ERR_NOT_FOUND;
    }

    // Increment transaction count
    dev->transaction_count++;

    // Clear consecutive failures
    dev->consecutive_failures = 0;

    // Increment consecutive successes
    dev->consecutive_successes++;

    // Check for recovery
    if (!dev->is_online && dev->consecutive_successes >= I2C_RECOVERY_THRESHOLD) {
        dev->is_online = true;
        give_mutex();

        // Publish recovery event (outside mutex)
        publish_recovery_event(addr);
        return ESP_OK;
    }

    give_mutex();
    return ESP_OK;
}

esp_err_t i2c_health_record_failure(i2c_port_t port, uint8_t addr)
{
    (void)port;  // Currently not used for lookup, just addr

    if (!s_i2c_health.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!take_mutex()) {
        return ESP_ERR_TIMEOUT;
    }

    I2CDeviceHealth* dev = find_device(addr);
    if (dev == NULL) {
        give_mutex();
        return ESP_ERR_NOT_FOUND;
    }

    // Increment failure counts
    dev->failure_count++;
    dev->consecutive_failures++;

    // Clear consecutive successes
    dev->consecutive_successes = 0;

    bool was_online = dev->is_online;
    bool now_offline = false;

    // Check threshold for going offline
    if (dev->is_online && dev->consecutive_failures >= I2C_FAILURE_THRESHOLD) {
        dev->is_online = false;
        now_offline = true;
    }

    give_mutex();

    // Increment error count via error_manager (outside mutex)
    // Story 4-10 integration: ERR_CAT_I2C
    if (error_manager_is_initialized()) {
        error_manager_increment(ERR_CAT_I2C, 0xFF, false);  // System-wide, no event yet
    }

    // Publish degradation event if just went offline
    if (was_online && now_offline) {
        publish_degradation_event(addr);

        // Publish system-wide I2C error event
        if (error_manager_is_initialized()) {
            error_manager_publish_error(ERR_CAT_I2C, 0xFF, ERR_I2C_FAILURE);
        }
    }

    return ESP_OK;
}

// ----------------------------------------------------------------------------
// Status Query Functions
// ----------------------------------------------------------------------------

esp_err_t i2c_health_get_bus_status(i2c_port_t port, I2CBusHealth *health)
{
    if (health == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_i2c_health.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!take_mutex()) {
        return ESP_ERR_TIMEOUT;
    }

    // Initialize
    memset(health, 0, sizeof(*health));
    health->status = I2C_HEALTH_OK;

    // Aggregate stats for devices on this port
    for (size_t i = 0; i < s_i2c_health.device_count; i++) {
        I2CDeviceHealth* dev = &s_i2c_health.devices[i];
        if (dev->port == port) {
            health->success_count += dev->transaction_count;
            health->failure_count += dev->failure_count;
            health->total_device_count++;

            if (dev->is_online) {
                health->online_device_count++;
            } else {
                health->status = I2C_HEALTH_ERROR;
            }

            // WARN if any failures but still online
            if (dev->failure_count > 0 && dev->is_online &&
                health->status == I2C_HEALTH_OK) {
                health->status = I2C_HEALTH_WARN;
            }
        }
    }

    give_mutex();
    return ESP_OK;
}

bool i2c_health_is_device_online(uint8_t addr)
{
    if (!s_i2c_health.initialized) {
        return true;  // Fail-safe: assume online if not initialized
    }

    // No mutex needed for simple read
    I2CDeviceHealth* dev = find_device(addr);
    if (dev == NULL) {
        return true;  // Unknown device assumed online
    }

    return dev->is_online;
}

esp_err_t i2c_health_get_device_status(uint8_t addr, I2CDeviceHealth *health)
{
    if (health == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_i2c_health.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!take_mutex()) {
        return ESP_ERR_TIMEOUT;
    }

    I2CDeviceHealth* dev = find_device(addr);
    if (dev == NULL) {
        give_mutex();
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(health, dev, sizeof(*health));

    give_mutex();
    return ESP_OK;
}

esp_err_t i2c_health_get_all_devices(I2CDeviceHealth *devices, size_t max_devices, size_t *count)
{
    if (devices == NULL || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_i2c_health.initialized) {
        *count = 0;
        return ESP_ERR_INVALID_STATE;
    }

    if (!take_mutex()) {
        *count = 0;
        return ESP_ERR_TIMEOUT;
    }

    size_t copy_count = s_i2c_health.device_count;
    if (copy_count > max_devices) {
        copy_count = max_devices;
    }

    memcpy(devices, s_i2c_health.devices, copy_count * sizeof(I2CDeviceHealth));
    *count = copy_count;

    give_mutex();
    return ESP_OK;
}

const char* i2c_health_get_device_name(uint8_t addr)
{
    if (!s_i2c_health.initialized) {
        return "UNKNOWN";
    }

    I2CDeviceHealth* dev = find_device(addr);
    if (dev == NULL) {
        return "UNKNOWN";
    }

    return dev->name;
}

// ----------------------------------------------------------------------------
// Device Registration
// ----------------------------------------------------------------------------

esp_err_t i2c_health_register_device(i2c_port_t port, uint8_t addr, const char *name)
{
    if (name == NULL || addr == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_i2c_health.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!take_mutex()) {
        return ESP_ERR_TIMEOUT;
    }

    // Check if already registered
    I2CDeviceHealth* existing = find_device(addr);
    if (existing != NULL) {
        give_mutex();
        ESP_LOGD(TAG, "Device 0x%02X already registered", addr);
        return ESP_OK;
    }

    // Check capacity
    if (s_i2c_health.device_count >= I2C_MAX_DEVICES) {
        give_mutex();
        ESP_LOGE(TAG, "Max devices reached, cannot register 0x%02X", addr);
        return ESP_ERR_NO_MEM;
    }

    // Add new device
    I2CDeviceHealth* dev = &s_i2c_health.devices[s_i2c_health.device_count];
    dev->address = addr;
    dev->port = port;
    strncpy(dev->name, name, I2C_DEVICE_NAME_LEN - 1);
    dev->name[I2C_DEVICE_NAME_LEN - 1] = '\0';
    dev->is_online = true;
    dev->transaction_count = 0;
    dev->failure_count = 0;
    dev->consecutive_failures = 0;
    dev->consecutive_successes = 0;

    s_i2c_health.device_count++;

    give_mutex();

    ESP_LOGI(TAG, "Registered device 0x%02X (%s) on I2C%d", addr, name, port);
    return ESP_OK;
}

// ----------------------------------------------------------------------------
// Test Functions
// ----------------------------------------------------------------------------

void i2c_health_inject_offline(uint8_t addr, bool offline)
{
    if (!s_i2c_health.initialized) {
        return;
    }

    I2CDeviceHealth* dev = find_device(addr);
    if (dev != NULL) {
        dev->is_online = !offline;
        ESP_LOGI(TAG, "TEST: Device 0x%02X set to %s", addr, offline ? "OFFLINE" : "ONLINE");
    }
}
