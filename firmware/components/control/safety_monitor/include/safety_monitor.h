/**
 * @file safety_monitor.h
 * @brief Safety Monitor API - Limit switch monitoring and safety interlocks
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-2: Limit Switch Monitoring
 *       - Monitors 14 limit switches (2 per axis for 7 axes, E has none)
 *       - Interrupt-driven detection via MCP23017 #0
 *       - Debounce filtering and polarity inversion support
 *       - Provides LIM command response data
 *
 * @see config_i2c.h for limit switch pin mappings
 * @see mcp23017_wrapper.h for I2C interface
 */

#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup safety_monitor Safety Monitor
 * @brief Limit switch monitoring and safety interlocks
 * @{
 */

/**
 * @defgroup limit_state Limit Switch State
 * @brief Limit switch state representation
 * @{
 */

/**
 * @brief Number of axes with limit switches
 *
 * 8 axes total (X,Y,Z,A,B,C,D,E), but E has no physical limits.
 */
#define SAFETY_NUM_AXES         8

/**
 * @brief E axis index (no physical limit switches)
 */
#define SAFETY_AXIS_E           7

/**
 * @brief Limit switch polarity - Normally Open
 */
#define LIMIT_POLARITY_NO       0

/**
 * @brief Limit switch polarity - Normally Closed
 */
#define LIMIT_POLARITY_NC       1

/**
 * @brief Limit switch state for a single axis
 */
typedef struct {
    bool min_active;    /**< MIN limit switch active (after polarity inversion) */
    bool max_active;    /**< MAX limit switch active (after polarity inversion) */
} axis_limit_state_t;

/** @} */ // end limit_state

/**
 * @defgroup safety_api Safety Monitor API
 * @brief Initialization and limit switch queries
 * @{
 */

/**
 * @brief Initialize safety monitor
 *
 * Performs the following:
 * - Initializes limit state cache
 * - Registers for MCP23017 interrupt notifications
 * - Installs ISR handlers (if not already installed)
 * - Starts safety_monitor_task at priority 24 on Core 0
 *
 * @pre mcp23017_wrapper_init() must be called first
 *
 * @return esp_err_t
 *         - ESP_OK: Initialized successfully
 *         - ESP_ERR_INVALID_STATE: Already initialized or MCP not ready
 *         - ESP_ERR_NO_MEM: Task creation failed
 */
esp_err_t safety_monitor_init(void);

/**
 * @brief Get cached limit state for all axes
 *
 * Returns a 16-bit value with 2 bits per axis:
 * - Bit 0: X MIN, Bit 1: X MAX
 * - Bit 2: Y MIN, Bit 3: Y MAX
 * - ... etc up to E axis
 *
 * E axis bits are always 0 (no physical switches).
 *
 * @return uint16_t Cached limit state (2 bits per axis, 8 axes)
 *
 * @note Returns cached value - updated on interrupt or poll.
 *       Call safety_monitor_update_cache() for fresh read.
 */
uint16_t safety_monitor_get_limit_state(void);

/**
 * @brief Get limit state for a single axis
 *
 * @param[in] axis Axis index (0-7 for X-E)
 * @param[out] min_active Pointer to store MIN limit state (true=active)
 * @param[out] max_active Pointer to store MAX limit state (true=active)
 *
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid axis or NULL pointers
 *
 * @note For E axis (7), always returns min_active=false, max_active=false
 */
esp_err_t safety_monitor_get_axis_limits(uint8_t axis, bool* min_active, bool* max_active);

/**
 * @brief Force update of limit state cache from hardware
 *
 * Reads MCP23017 #0 and updates the cached limit state.
 * Normally called automatically on interrupt, but can be called
 * manually for diagnostics or polling mode.
 *
 * @return esp_err_t
 *         - ESP_OK: Cache updated successfully
 *         - ESP_ERR_TIMEOUT: I2C read timeout
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t safety_monitor_update_cache(void);

/**
 * @brief Check if safety monitor is initialized
 *
 * @return true if initialized, false otherwise
 */
bool safety_monitor_is_initialized(void);

/**
 * @brief Set limit switch polarity for an axis
 *
 * @param[in] axis Axis index (0-7)
 * @param[in] min_polarity Polarity for MIN switch (LIMIT_POLARITY_NO or LIMIT_POLARITY_NC)
 * @param[in] max_polarity Polarity for MAX switch (LIMIT_POLARITY_NO or LIMIT_POLARITY_NC)
 *
 * @return esp_err_t
 *         - ESP_OK: Polarity set successfully
 *         - ESP_ERR_INVALID_ARG: Invalid axis or polarity value
 */
esp_err_t safety_monitor_set_polarity(uint8_t axis, uint8_t min_polarity, uint8_t max_polarity);

/**
 * @brief Get limit switch polarity for an axis
 *
 * @param[in] axis Axis index (0-7)
 * @param[out] min_polarity Pointer to store MIN switch polarity
 * @param[out] max_polarity Pointer to store MAX switch polarity
 *
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid axis or NULL pointers
 */
esp_err_t safety_monitor_get_polarity(uint8_t axis, uint8_t* min_polarity, uint8_t* max_polarity);

/**
 * @brief Deinitialize safety monitor
 *
 * Stops the safety monitor task and releases resources.
 * Safe to call if not initialized.
 *
 * @return esp_err_t
 *         - ESP_OK: Deinitialized successfully
 */
esp_err_t safety_monitor_deinit(void);

/** @} */ // end safety_api

/**
 * @defgroup lim_handler LIM Command Handler
 * @brief Command handler for LIM command
 * @{
 */

/**
 * @brief Register LIM command handler
 *
 * Registers the CMD_LIM handler with the command executor.
 * Called during safety_monitor_init() or motor_system_init().
 *
 * @return esp_err_t
 *         - ESP_OK: Handler registered successfully
 *         - ESP_ERR_NO_MEM: Command table full
 */
esp_err_t lim_handler_register(void);

/** @} */ // end lim_handler

/**
 * @defgroup safety_test Test/Debug Functions
 * @brief Functions for testing without physical switches
 * @{
 */

/**
 * @brief Inject a simulated limit state for testing
 *
 * Bypasses hardware read and sets the limit state cache directly.
 * Useful for testing LIM command and integration without physical switches.
 *
 * @param[in] simulated_state 16-bit state to inject (2 bits per axis)
 *
 * @note Only available when safety_monitor is initialized.
 *       Call safety_monitor_update_cache() to restore hardware state.
 */
void safety_monitor_inject_test_state(uint16_t simulated_state);

/** @} */ // end safety_test

/**
 * @defgroup estop Emergency Stop System (Story 4-4)
 * @brief E-stop ISR, mode management, and recovery
 * @{
 */

/**
 * @brief E-stop notification bit for safety_monitor_task
 *
 * Set by E-stop ISR to notify task of E-stop activation.
 * Uses bit 4 (bits 0-3 used by MCP_NOTIFY_* in mcp23017_wrapper.h).
 */
#define NOTIFY_ESTOP            (1 << 4)

/**
 * @brief Check if E-stop is currently active
 *
 * @return true if system is in E-stop mode
 * @return false if system is not in E-stop mode
 */
bool safety_monitor_is_estop_active(void);

/**
 * @brief Get E-stop button physical state
 *
 * Reads the GPIO_E_STOP pin directly to determine if the
 * physical button is still pressed.
 *
 * @return true if button is pressed (GPIO low)
 * @return false if button is released (GPIO high)
 */
bool safety_monitor_is_estop_button_pressed(void);

/**
 * @brief Attempt to reset E-stop and return to normal operation
 *
 * Only succeeds if:
 * 1. System is in E-stop mode
 * 2. E-stop button is released (GPIO high)
 *
 * On success:
 * - Transitions system mode to IDLE
 * - Re-enables shift register outputs
 * - Publishes EVENT ESTOP INACTIVE
 *
 * @return esp_err_t
 *         - ESP_OK: E-stop cleared successfully
 *         - ESP_ERR_INVALID_STATE: Not in E-stop mode or button still pressed
 */
esp_err_t safety_monitor_reset_estop(void);

/**
 * @brief Mark all axes as unhomed
 *
 * Called during E-stop activation to indicate position may be lost.
 * All axes require re-homing before trusted positioning.
 */
void safety_monitor_mark_all_unhomed(void);

/**
 * @brief Simulate E-stop button press for testing
 *
 * Triggers the same sequence as physical button press:
 * - Calls sr_emergency_disable_all()
 * - Notifies safety_monitor_task
 * - Results in STATE_ESTOP and EVENT ESTOP ACTIVE
 *
 * @note For testing only - does not require physical button
 */
void safety_monitor_inject_estop(void);

/**
 * @brief Set simulated E-stop button state for testing
 *
 * When test mode is active, this overrides the physical GPIO reading.
 * Use to test RST command without physical button.
 *
 * @param[in] pressed true = button pressed (RST will fail)
 *                    false = button released (RST will succeed)
 */
void safety_monitor_set_test_button_state(bool pressed);

/**
 * @brief Enable/disable E-stop test mode
 *
 * When enabled, safety_monitor_is_estop_button_pressed() returns
 * the simulated state instead of reading GPIO.
 *
 * @param[in] enable true to enable test mode, false for normal operation
 */
void safety_monitor_set_estop_test_mode(bool enable);

/** @} */ // end estop

/**
 * @defgroup limit_motion_stop Limit Switch Motion Stop (Story 4-3)
 * @brief Motion stop integration and direction blocking
 * @{
 */

/**
 * @brief End switch operating mode (per-switch configuration)
 *
 * Each limit switch (MIN and MAX) can be configured independently.
 * Uses EndSwitchMode from config_commands.h:
 * - END_SWITCH_MODE_NONE: Switch not connected or disabled
 * - END_SWITCH_MODE_HARD_STOP: Stop motion immediately, block direction
 * - END_SWITCH_MODE_RESTRICT: Stop motion, allow move away from limit
 * - END_SWITCH_MODE_EVENT_ONLY: Fire event only, don't stop motion
 */

/**
 * @brief Set end switch mode for a specific switch
 *
 * Configures how the safety monitor responds when this limit switch activates.
 *
 * @param[in] axis Axis index (0-7)
 * @param[in] is_max true for MAX switch, false for MIN switch
 * @param[in] mode EndSwitchMode value
 *
 * @return esp_err_t
 *         - ESP_OK: Mode set successfully
 *         - ESP_ERR_INVALID_ARG: Invalid axis or mode
 */
esp_err_t safety_monitor_set_endswitch_mode(uint8_t axis, bool is_max, uint8_t mode);

/**
 * @brief Get end switch mode for a specific switch
 *
 * @param[in] axis Axis index (0-7)
 * @param[in] is_max true for MAX switch, false for MIN switch
 * @param[out] mode Pointer to store EndSwitchMode value
 *
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid axis or NULL pointer
 */
esp_err_t safety_monitor_get_endswitch_mode(uint8_t axis, bool is_max, uint8_t* mode);

/**
 * @brief Stop axis motion due to limit activation
 *
 * Called internally when limit switch activates with HARD_STOP or RESTRICT mode.
 * Calls motion_controller->stopAxis() with controlled deceleration.
 *
 * @param[in] axis Axis index (0-7)
 *
 * @return esp_err_t
 *         - ESP_OK: Motion stop initiated
 *         - ESP_ERR_INVALID_ARG: Invalid axis
 *         - ESP_ERR_INVALID_STATE: Motion controller not initialized
 */
esp_err_t safety_monitor_stop_axis_motion(uint8_t axis);

/**
 * @brief Check if motion in a direction is blocked by active limit
 *
 * Per ADR-022, always stop first on limit trigger. This function checks
 * if subsequent motion in a direction is blocked.
 *
 * @param[in] axis Axis index (0-7)
 * @param[in] direction Movement direction: +1 for positive/MAX, -1 for negative/MIN
 *
 * @return true if motion in that direction is blocked by active limit
 * @return false if motion is allowed
 *
 * @note Returns false for invalid axis or direction = 0
 */
bool safety_monitor_is_direction_blocked(uint8_t axis, int8_t direction);

/**
 * @brief Get axis fault state
 *
 * Returns true if axis is in FAULT state (e.g., both limits active).
 *
 * @param[in] axis Axis index (0-7)
 *
 * @return true if axis is faulted
 * @return false if axis is normal or axis is invalid
 */
bool safety_monitor_is_axis_faulted(uint8_t axis);

/**
 * @brief Clear axis fault state
 *
 * Clears the FAULT state for an axis if the fault condition has been resolved.
 * For BOTH_LIMITS fault, both limits must be inactive before clearing.
 *
 * @param[in] axis Axis index (0-7)
 *
 * @return esp_err_t
 *         - ESP_OK: Fault cleared
 *         - ESP_ERR_INVALID_ARG: Invalid axis
 *         - ESP_ERR_INVALID_STATE: Fault condition still present
 */
esp_err_t safety_monitor_clear_fault(uint8_t axis);

/**
 * @brief Limit change callback function type
 *
 * Called when limit switch state changes. Invoked from safety_monitor_task context.
 *
 * @param[in] axis Axis that changed
 * @param[in] min_active New MIN limit state
 * @param[in] max_active New MAX limit state
 * @param[in] ctx User context pointer
 */
typedef void (*safety_limit_callback_t)(uint8_t axis, bool min_active, bool max_active, void* ctx);

/**
 * @brief Register callback for limit state changes
 *
 * @param[in] callback Function to call on limit changes
 * @param[in] ctx User context pointer passed to callback
 *
 * @return esp_err_t
 *         - ESP_OK: Callback registered
 *         - ESP_ERR_INVALID_ARG: callback is NULL
 *         - ESP_ERR_NO_MEM: Maximum callbacks registered
 */
esp_err_t safety_monitor_register_limit_callback(safety_limit_callback_t callback, void* ctx);

/** @} */ // end limit_motion_stop

/**
 * @defgroup safety_degraded Graceful Degradation (Story 4-11)
 * @brief I2C device offline handling
 * @{
 */

/**
 * @brief Check if limit monitoring is degraded
 *
 * Returns true if MCP23017 #0 (limit switches) is offline.
 * When degraded, limit switch readings may not be reliable.
 *
 * @return true if limit monitoring is degraded
 * @return false if limit monitoring is operational
 */
bool safety_monitor_is_limits_degraded(void);

/**
 * @brief Check if feedback monitoring is degraded
 *
 * Returns true if MCP23017 #1 (InPos/ALARM) is offline.
 * When degraded, InPos and ALARM readings may not be reliable.
 *
 * @return true if feedback monitoring is degraded
 * @return false if feedback monitoring is operational
 */
bool safety_monitor_is_feedback_degraded(void);

/** @} */ // end safety_degraded

/** @} */ // end safety_monitor

#ifdef __cplusplus
}
#endif

#endif // SAFETY_MONITOR_H
