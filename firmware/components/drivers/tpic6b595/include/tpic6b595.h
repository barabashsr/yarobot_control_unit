/**
 * @file tpic6b595.h
 * @brief TPIC6B595N Shift Register Driver API
 * @author YaRobot Team
 * @date 2025
 *
 * @note Controls 5x TPIC6B595N shift register chain (40 bits) via SPI.
 *       Provides per-axis direction, enable, brake, and alarm clear control,
 *       plus 8 general-purpose outputs.
 *
 * @warning Thread-safe except sr_emergency_disable_all() which is ISR-safe.
 */

#ifndef TPIC6B595_H
#define TPIC6B595_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup tpic6b595 TPIC6B595N Shift Register Driver
 * @brief Driver for 5x TPIC6B595N daisy-chained shift registers
 * @{
 */

/**
 * @brief Number of motor axes supported (X, Y, Z, A, B, C, D, E)
 */
#define SR_NUM_AXES 8

/**
 * @brief Number of general-purpose output pins
 */
#define SR_NUM_GP_OUTPUTS 8

/**
 * @brief Axis indices for sr_set_* functions
 */
typedef enum {
    SR_AXIS_X = 0,
    SR_AXIS_Y = 1,
    SR_AXIS_Z = 2,
    SR_AXIS_A = 3,
    SR_AXIS_B = 4,
    SR_AXIS_C = 5,
    SR_AXIS_D = 6,
    SR_AXIS_E = 7
} sr_axis_t;

/**
 * @brief Initialize the shift register driver
 *
 * Configures SPI via HAL layer, sets up GPIO for OE control,
 * initializes shadow register to safe state (all bits 0),
 * and latches the safe state to hardware.
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 *
 * @note Must be called before any other sr_* functions
 * @note OE remains LOW (outputs enabled) after init
 */
esp_err_t sr_init(void);

/**
 * @brief Set motor direction for an axis
 *
 * @param[in] axis Axis index (0-7, use sr_axis_t enum)
 * @param[in] forward true for forward, false for reverse
 *
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if axis > 7
 *
 * @note Only updates shadow register; call sr_update() to latch to hardware
 */
esp_err_t sr_set_direction(uint8_t axis, bool forward);

/**
 * @brief Set motor enable for an axis
 *
 * @param[in] axis Axis index (0-7, use sr_axis_t enum)
 * @param[in] enable true to enable motor, false to disable
 *
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if axis > 7
 *
 * @note Only updates shadow register; call sr_update() to latch to hardware
 */
esp_err_t sr_set_enable(uint8_t axis, bool enable);

/**
 * @brief Set brake release for an axis
 *
 * @param[in] axis Axis index (0-6 for X-D, E has no brake)
 * @param[in] release true to release brake, false to engage
 *
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if axis > 6
 *
 * @note Only updates shadow register; call sr_update() to latch to hardware
 * @note E-axis (7) has no brake - returns ESP_ERR_INVALID_ARG
 */
esp_err_t sr_set_brake(uint8_t axis, bool release);

/**
 * @brief Set alarm clear signal for an axis
 *
 * @param[in] axis Axis index (0-6 for X-D)
 * @param[in] active true to assert alarm clear, false to deassert
 *
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if axis > 6
 *
 * @note Only updates shadow register; call sr_update() to latch to hardware
 * @note E-axis (7) has no alarm clear - returns ESP_ERR_INVALID_ARG
 */
esp_err_t sr_set_alarm_clear(uint8_t axis, bool active);

/**
 * @brief Set general-purpose output pin level
 *
 * @param[in] pin GP output index (0-7)
 * @param[in] level true for HIGH, false for LOW
 *
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if pin > 7
 *
 * @note Only updates shadow register; call sr_update() to latch to hardware
 */
esp_err_t sr_set_gp_output(uint8_t pin, bool level);

/**
 * @brief Latch shadow register to shift register outputs
 *
 * Transfers all 40 bits via SPI and pulses LATCH to update outputs.
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 *
 * @note Thread-safe (uses mutex)
 */
esp_err_t sr_update(void);

/**
 * @brief Get current shadow register state
 *
 * @return uint64_t Current 40-bit shadow register value (bits 0-39 valid)
 */
uint64_t sr_get_state(void);

/**
 * @brief Emergency disable all outputs
 *
 * Sets OE HIGH (tristates outputs), clears shadow register to safe state,
 * and shifts zeros to hardware. Completes within 100us.
 *
 * @note ISR-safe: Does NOT use mutex or blocking RTOS calls
 * @note Outputs remain tristated until sr_init() or explicit recovery
 * @warning Call from ISR or high-priority safety task only
 */
void sr_emergency_disable_all(void);

/**
 * @brief Re-enable outputs after emergency disable
 *
 * Sets OE LOW to enable outputs. Shadow register retains safe state
 * until explicitly modified.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sr_enable_outputs(void);

/**
 * @brief Check if shift register driver is initialized
 *
 * @return true if initialized, false otherwise
 */
bool sr_is_initialized(void);

/** @} */ // end tpic6b595

#ifdef __cplusplus
}
#endif

#endif // TPIC6B595_H
