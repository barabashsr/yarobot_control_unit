/**
 * @file floating_switch.h
 * @brief Floating Switch Handler API - C-axis object width measurement
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-9: C-Axis Floating Switch (Object Width Measurement)
 *       - Detects floating switch (C_LIMIT_MAX) during closing motion
 *       - Calculates object width from position at contact
 *       - Publishes EVENT WIDTH C <value> on measurement
 *       - Provides WIDTH command response data
 *
 * The floating switch on C axis (picker jaw) is used for automatic object
 * size detection. Unlike a hard limit, the floating switch:
 * - Only triggers measurement during closing motion
 * - Stops motion gracefully (no error state)
 * - Calculates width from current position
 *
 * @see config_i2c.h for MCP0_C_LIMIT_MAX pin mapping
 * @see safety_monitor.h for limit callback mechanism
 */

#ifndef FLOATING_SWITCH_H
#define FLOATING_SWITCH_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup floating_switch Floating Switch Handler
 * @brief C-axis object width measurement via floating switch
 * @{
 */

/**
 * @brief No measurement indicator value
 *
 * Returned by floating_switch_get_width() when no measurement has been taken.
 */
#define FLOATING_SWITCH_NO_MEASUREMENT  (-1.0f)

/**
 * @defgroup floating_switch_api Floating Switch API
 * @brief Initialization and width measurement queries
 * @{
 */

/**
 * @brief Initialize floating switch handler
 *
 * Performs the following:
 * - Registers limit callback for C axis MAX limit changes
 * - Initializes measurement state (no measurement)
 *
 * @pre safety_monitor_init() must be called first
 * @pre motion_controller must be initialized
 *
 * @return esp_err_t
 *         - ESP_OK: Initialized successfully
 *         - ESP_ERR_INVALID_STATE: Already initialized or dependencies not ready
 */
esp_err_t floating_switch_init(void);

/**
 * @brief Check if floating switch handler is initialized
 *
 * @return true if initialized, false otherwise
 */
bool floating_switch_is_initialized(void);

/**
 * @brief Get last measured width
 *
 * Returns the width measured during the last floating switch trigger.
 *
 * @return Width in configured units (mm by default), or
 *         FLOATING_SWITCH_NO_MEASUREMENT (-1.0) if no measurement taken
 */
float floating_switch_get_width(void);

/**
 * @brief Check if a valid measurement exists
 *
 * @return true if a width measurement has been taken since boot/reset
 */
bool floating_switch_has_measurement(void);

/**
 * @brief Clear stored width measurement
 *
 * Called internally when C axis starts a new closing motion.
 * Can also be called externally to reset measurement state.
 */
void floating_switch_clear_measurement(void);

/**
 * @brief Notify floating switch handler of new C axis motion
 *
 * Called by motion controller when C axis starts moving.
 * If motion is in closing direction (negative), clears previous measurement.
 *
 * @param[in] direction Motion direction: -1 for closing, +1 for opening
 */
void floating_switch_on_motion_start(int8_t direction);

/**
 * @brief Deinitialize floating switch handler
 *
 * Unregisters limit callback and releases resources.
 * Safe to call if not initialized.
 *
 * @return esp_err_t
 *         - ESP_OK: Deinitialized successfully
 */
esp_err_t floating_switch_deinit(void);

/** @} */ // end floating_switch_api

/** @} */ // end floating_switch

#ifdef __cplusplus
}
#endif

#endif // FLOATING_SWITCH_H
