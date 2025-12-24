/**
 * @file position_loss.h
 * @brief Position Loss Detector API - Tracks axis position validity
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-6: Position Loss Detection
 *       - Tracks POSLOS (position lost) state for servo axes (X,Y,Z,A,B)
 *       - Tracks HOMED state for all axes
 *       - Sets POSLOS on boot and E-stop (servos may have coasted)
 *       - Clears POSLOS via POSOK command or successful homing
 *       - Blocks motion commands when POSLOS is set
 *
 * @see config_commands.h for CMD_POSOK, ERR_POSLOS, EVT_POSLOS
 * @see brake_controller.h for BRAKE_NUM_AXES (same servo axes)
 */

#ifndef POSITION_LOSS_H
#define POSITION_LOSS_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup position_loss Position Loss Detector
 * @brief Axis position validity tracking
 * @{
 */

/**
 * @brief Number of servo axes with POSLOS tracking
 *
 * Servo axes X, Y, Z, A, B (0-4) have position loss detection.
 * Stepper axes C, D (5-6) are open-loop, use HOMED only.
 * Discrete axis E (7) has no position tracking.
 */
#define POSLOS_NUM_SERVO_AXES   5

/**
 * @brief Number of total axes for homing tracking
 */
#define POSLOS_NUM_ALL_AXES     8

/**
 * @brief Axis indices for position loss
 */
#define POSLOS_AXIS_X           0
#define POSLOS_AXIS_Y           1
#define POSLOS_AXIS_Z           2
#define POSLOS_AXIS_A           3
#define POSLOS_AXIS_B           4

/**
 * @defgroup poslos_api Position Loss API
 * @brief Initialization and state management functions
 * @{
 */

/**
 * @brief Initialize position loss detector
 *
 * Performs the following (AC1):
 * - Sets POSLOS flag for all servo axes (X,Y,Z,A,B)
 * - Sets UNHOMED for all axes (X-E)
 * - Publishes EVENT POSLOS for each servo axis
 *
 * @return esp_err_t
 *         - ESP_OK: Initialized successfully
 *         - ESP_ERR_INVALID_STATE: Already initialized
 */
esp_err_t position_loss_init(void);

/**
 * @brief Check if position loss detector is initialized
 *
 * @return true if initialized, false otherwise
 */
bool position_loss_is_initialized(void);

/**
 * @brief Set POSLOS flag for a servo axis
 *
 * @param[in] axis Axis index (0-4 for X-B servo axes only)
 * @param[in] lost true to set POSLOS, false to clear
 *
 * @return esp_err_t
 *         - ESP_OK: Flag set/cleared successfully
 *         - ESP_ERR_INVALID_ARG: Invalid axis (not a servo axis)
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t position_loss_set(uint8_t axis, bool lost);

/**
 * @brief Get POSLOS state for an axis
 *
 * @param[in] axis Axis index (0-4 for servo, 5-7 always false)
 *
 * @return true if position is lost (POSLOS set)
 * @return false if position is valid or axis is not a servo
 *
 * @note Returns false for stepper axes (C,D) and discrete (E)
 */
bool position_loss_get(uint8_t axis);

/**
 * @brief Get POSLOS state for all servo axes as bitmask
 *
 * @return uint8_t 5-bit bitmask where bit N = axis N POSLOS state
 *         Bit 0=X, 1=Y, 2=Z, 3=A, 4=B (1=lost, 0=OK)
 */
uint8_t position_loss_get_all(void);

/**
 * @brief Handle E-stop event (AC2)
 *
 * Sets POSLOS for all servo axes (servos may have coasted during stop).
 * Publishes EVENT POSLOS for each affected axis.
 * Stepper axes are NOT marked POSLOS (they stop immediately).
 */
void position_loss_on_estop(void);

/**
 * @brief Handle homing complete (AC8)
 *
 * Clears POSLOS flag and sets HOMED flag for the axis.
 * No POSOK required after successful homing.
 *
 * @param[in] axis Axis index (0-7)
 *
 * @note Hook for Story 4.13 (Homing system)
 */
void position_loss_on_homing_complete(uint8_t axis);

/**
 * @brief Clear POSLOS for a single axis via POSOK command (AC4)
 *
 * Clears POSLOS flag and returns axis state to IDLE.
 * User accepts responsibility for position accuracy.
 *
 * @param[in] axis Axis index (0-4 for servo axes)
 *
 * @return esp_err_t
 *         - ESP_OK: POSLOS cleared
 *         - ESP_ERR_INVALID_ARG: Invalid axis (not a servo)
 *         - ESP_ERR_INVALID_STATE: Axis not in POSLOS state
 */
esp_err_t position_loss_acknowledge(uint8_t axis);

/**
 * @brief Clear POSLOS for all servo axes via POSOK command (AC5)
 *
 * Clears POSLOS flag for all servo axes (X,Y,Z,A,B).
 *
 * @return esp_err_t
 *         - ESP_OK: All POSLOS flags cleared
 */
esp_err_t position_loss_acknowledge_all(void);

/**
 * @brief Check if axis is a servo (has POSLOS capability)
 *
 * @param[in] axis Axis index (0-7)
 *
 * @return true if axis is a servo (X,Y,Z,A,B)
 * @return false if axis is a stepper (C,D) or discrete (E)
 */
bool position_loss_is_servo_axis(uint8_t axis);

/**
 * @brief Get homed state for an axis
 *
 * @param[in] axis Axis index (0-7)
 *
 * @return true if axis is homed
 * @return false if axis is unhomed or invalid
 */
bool position_loss_is_homed(uint8_t axis);

/**
 * @brief Mark all axes as unhomed
 *
 * Called during E-stop and power-on to reset homing state.
 */
void position_loss_mark_all_unhomed(void);

/** @} */ // end poslos_api

/**
 * @defgroup poslos_test Test/Debug Functions
 * @brief Functions for testing position loss detection
 * @{
 */

/**
 * @brief Inject POSLOS state for testing
 *
 * @param[in] axis Axis index (0-4 for servo axes)
 * @param[in] lost true to simulate POSLOS, false to clear
 */
void position_loss_inject_test_state(uint8_t axis, bool lost);

/**
 * @brief Deinitialize position loss detector
 *
 * Resets all state. Safe to call if not initialized.
 *
 * @return esp_err_t
 *         - ESP_OK: Deinitialized successfully
 */
esp_err_t position_loss_deinit(void);

/** @} */ // end poslos_test

/** @} */ // end position_loss

#ifdef __cplusplus
}
#endif

#endif // POSITION_LOSS_H
