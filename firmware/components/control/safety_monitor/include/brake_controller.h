/**
 * @file brake_controller.h
 * @brief Brake Controller API - Per-axis brake management with configurable strategies
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-5: Brake Control System
 *       - Per-axis brake strategies: BRAKE_ON_DISABLE, BRAKE_ON_ESTOP, BRAKE_ON_IDLE, BRAKE_MANUAL
 *       - Timing-correct engage/release sequences
 *       - Idle timeout with auto-engage
 *       - EVENT BRAKE notifications
 *
 * @see config_defaults.h for BrakeStrategy enum
 * @see config_timing.h for TIMING_BRAKE_ENGAGE_MS, TIMING_BRAKE_RELEASE_MS, TIMING_IDLE_TIMEOUT_S
 */

#ifndef BRAKE_CONTROLLER_H
#define BRAKE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "config_defaults.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup brake_controller Brake Controller
 * @brief Per-axis brake management with configurable strategies
 * @{
 */

/**
 * @brief Number of axes with brake hardware
 *
 * Servo axes X, Y, Z, A, B have brakes (0-4).
 * Stepper axes C, D (5-6) and discrete axis E (7) have no brakes.
 */
#define BRAKE_NUM_AXES          5

/**
 * @brief First axis index without brake hardware (C axis)
 */
#define BRAKE_FIRST_NO_BRAKE    5

/**
 * @brief Axis indices
 */
#define BRAKE_AXIS_X            0
#define BRAKE_AXIS_Y            1
#define BRAKE_AXIS_Z            2
#define BRAKE_AXIS_A            3
#define BRAKE_AXIS_B            4

/**
 * @defgroup brake_api Brake Controller API
 * @brief Initialization and brake control functions
 * @{
 */

/**
 * @brief Initialize brake controller
 *
 * Performs the following:
 * - Initializes brake state tracking (all brakes engaged = safe state)
 * - Sets default strategies per axis (Z = BRAKE_ON_DISABLE, others = BRAKE_ON_ESTOP)
 * - Creates idle timeout timers for BRAKE_ON_IDLE axes
 *
 * @pre sr_init() must be called first (shift register driver)
 *
 * @return esp_err_t
 *         - ESP_OK: Initialized successfully
 *         - ESP_ERR_INVALID_STATE: Already initialized
 *         - ESP_ERR_NO_MEM: Timer creation failed
 */
esp_err_t brake_controller_init(void);

/**
 * @brief Check if brake controller is initialized
 *
 * @return true if initialized, false otherwise
 */
bool brake_controller_is_initialized(void);

/**
 * @brief Set brake state for an axis
 *
 * Engages or releases the brake with proper timing delays.
 * For BRAKE_ON_IDLE axes, resets the idle timer when releasing.
 *
 * @param[in] axis Axis index (0-4 for X-B)
 * @param[in] engage true to engage brake, false to release
 *
 * @return esp_err_t
 *         - ESP_OK: Brake state changed successfully
 *         - ESP_ERR_INVALID_ARG: Invalid axis (>= BRAKE_FIRST_NO_BRAKE)
 *         - ESP_ERR_INVALID_STATE: Not initialized
 *
 * @note Applies timing delay internally:
 *       - Engage: Updates SR immediately, delays TIMING_BRAKE_ENGAGE_MS
 *       - Release: Delays TIMING_BRAKE_RELEASE_MS after SR update
 * @note Does NOT call sr_update() - caller must call it after this function
 */
esp_err_t brake_set_state(uint8_t axis, bool engage);

/**
 * @brief Set brake state with timing delay
 *
 * Same as brake_set_state() but performs the full operation including
 * sr_update() and appropriate timing delays.
 *
 * @param[in] axis Axis index (0-4 for X-B)
 * @param[in] engage true to engage brake, false to release
 *
 * @return esp_err_t
 *         - ESP_OK: Brake state changed with timing
 *         - ESP_ERR_INVALID_ARG: Invalid axis
 *         - ESP_ERR_INVALID_STATE: Not initialized
 *
 * @note Blocking call - waits for timing delay to complete
 */
esp_err_t brake_set_state_with_timing(uint8_t axis, bool engage);

/**
 * @brief Get current brake state for an axis
 *
 * @param[in] axis Axis index (0-4 for X-B)
 *
 * @return true if brake is engaged, false if released
 *
 * @note Returns true (engaged) for invalid axes or if not initialized
 */
bool brake_get_state(uint8_t axis);

/**
 * @brief Get brake states for all servo axes as bitmask
 *
 * @return uint8_t Bitmask where bit N = axis N brake state (1=released, 0=engaged)
 *         Bits 0-4 for X, Y, Z, A, B. Bits 5-7 undefined.
 *
 * @note Follows shift register convention: 1=released, 0=engaged
 */
uint8_t brake_get_all_states(void);

/**
 * @brief Set brake strategy for an axis
 *
 * @param[in] axis Axis index (0-4 for X-B)
 * @param[in] strategy BrakeStrategy value
 *
 * @return esp_err_t
 *         - ESP_OK: Strategy set successfully
 *         - ESP_ERR_INVALID_ARG: Invalid axis or strategy
 */
esp_err_t brake_set_strategy(uint8_t axis, BrakeStrategy strategy);

/**
 * @brief Get brake strategy for an axis
 *
 * @param[in] axis Axis index (0-4 for X-B)
 *
 * @return BrakeStrategy Current strategy for the axis
 *
 * @note Returns BRAKE_ON_ESTOP for invalid axes
 */
BrakeStrategy brake_get_strategy(uint8_t axis);

/**
 * @brief Check if axis has brake hardware
 *
 * @param[in] axis Axis index (0-7)
 *
 * @return true if axis has brake (X, Y, Z, A, B)
 * @return false if axis has no brake (C, D, E) or invalid
 */
bool brake_has_hardware(uint8_t axis);

/** @} */ // end brake_api

/**
 * @defgroup brake_motion Motion Integration
 * @brief Functions for motion controller integration
 * @{
 */

/**
 * @brief Handle axis enable request (BRAKE_ON_DISABLE integration)
 *
 * Called by enable_handler before enabling motor. Performs brake sequence:
 * 1. Enable motor (caller responsibility)
 * 2. Wait TIMING_BRAKE_RELEASE_MS
 * 3. Release brake
 *
 * @param[in] axis Axis index (0-4 for X-B)
 *
 * @return esp_err_t
 *         - ESP_OK: Sequence completed
 *         - ESP_ERR_INVALID_ARG: Invalid axis
 *         - ESP_ERR_NOT_SUPPORTED: Strategy is not BRAKE_ON_DISABLE
 */
esp_err_t brake_on_axis_enable(uint8_t axis);

/**
 * @brief Handle axis disable request (BRAKE_ON_DISABLE integration)
 *
 * Called by enable_handler before disabling motor. Performs brake sequence:
 * 1. Engage brake
 * 2. Wait TIMING_BRAKE_ENGAGE_MS
 * 3. Disable motor (caller responsibility)
 *
 * @param[in] axis Axis index (0-4 for X-B)
 *
 * @return esp_err_t
 *         - ESP_OK: Sequence completed (brake engaged, ready for motor disable)
 *         - ESP_ERR_INVALID_ARG: Invalid axis
 *         - ESP_ERR_NOT_SUPPORTED: Strategy is not BRAKE_ON_DISABLE
 */
esp_err_t brake_on_axis_disable(uint8_t axis);

/**
 * @brief Notify brake controller of motion start
 *
 * For BRAKE_ON_IDLE axes:
 * - Releases brake if engaged (with timing delay)
 * - Resets idle timer
 *
 * @param[in] axis Axis index (0-4 for X-B)
 *
 * @return esp_err_t
 *         - ESP_OK: Ready for motion (brake released if needed)
 *         - ESP_ERR_INVALID_ARG: Invalid axis
 */
esp_err_t brake_on_motion_start(uint8_t axis);

/**
 * @brief Notify brake controller of motion stop
 *
 * For BRAKE_ON_IDLE axes:
 * - Starts idle timer (will engage brake after timeout)
 *
 * @param[in] axis Axis index (0-4 for X-B)
 */
void brake_on_motion_stop(uint8_t axis);

/** @} */ // end brake_motion

/**
 * @defgroup brake_test Test/Debug Functions
 * @brief Functions for testing brake control
 * @{
 */

/**
 * @brief Inject brake state for testing (bypasses hardware)
 *
 * @param[in] axis Axis index (0-4 for X-B)
 * @param[in] engaged true to simulate engaged, false for released
 */
void brake_inject_test_state(uint8_t axis, bool engaged);

/**
 * @brief Force idle timeout for testing
 *
 * Immediately triggers idle brake engagement for BRAKE_ON_IDLE axes
 * without waiting for the actual timeout.
 *
 * @param[in] axis Axis index (0-4 for X-B)
 */
void brake_force_idle_timeout(uint8_t axis);

/** @} */ // end brake_test

/**
 * @brief Deinitialize brake controller
 *
 * Stops idle timers and releases resources.
 *
 * @return esp_err_t
 *         - ESP_OK: Deinitialized successfully
 */
esp_err_t brake_controller_deinit(void);

/** @} */ // end brake_controller

#ifdef __cplusplus
}
#endif

#endif // BRAKE_CONTROLLER_H
