/**
 * @file servo_feedback.h
 * @brief Servo Feedback Reader API - InPos signal monitoring
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-8: Servo Feedback Processing (Basic InPos)
 *       Reads InPos (in-position) signals from servo drivers via MCP23017 #1 Port B.
 *       Servo axes X,Y,Z,A,B have InPos feedback; stepper axes C,D,E do not.
 *
 * @see config_i2c.h for MCP1_X_INPOS through MCP1_B_INPOS pin mappings
 */

#ifndef SERVO_FEEDBACK_H
#define SERVO_FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup servo_feedback Servo Feedback Reader
 * @brief InPos signal reading from servo drivers
 * @{
 */

/**
 * @brief Number of servo axes with InPos feedback (X,Y,Z,A,B)
 */
#define SERVO_FEEDBACK_NUM_AXES     5

/**
 * @brief Initialize servo feedback reader
 *
 * Must be called after mcp23017_wrapper_init().
 *
 * @return esp_err_t
 *         - ESP_OK: Initialized successfully
 *         - ESP_ERR_INVALID_STATE: mcp23017_wrapper not initialized
 */
esp_err_t servo_feedback_init(void);

/**
 * @brief Check if servo feedback reader is initialized
 *
 * @return true if initialized, false otherwise
 */
bool servo_feedback_is_initialized(void);

/**
 * @brief Get InPos state for a single axis
 *
 * Reads the InPos signal for the specified servo axis.
 * For stepper/discrete axes (C,D,E), returns false with ESP_ERR_NOT_SUPPORTED.
 *
 * @param[in] axis Axis index (AXIS_X through AXIS_B for servo axes)
 * @param[out] inpos Pointer to store InPos state (true = in position)
 *
 * @return esp_err_t
 *         - ESP_OK: Read successful
 *         - ESP_ERR_INVALID_STATE: Not initialized
 *         - ESP_ERR_INVALID_ARG: NULL pointer or invalid axis
 *         - ESP_ERR_NOT_SUPPORTED: Axis has no servo feedback (C,D,E)
 *         - ESP_ERR_TIMEOUT: I2C read timeout
 */
esp_err_t servo_feedback_get_inpos(uint8_t axis, bool *inpos);

/**
 * @brief Get InPos state for all servo axes
 *
 * Returns a 5-bit bitmap with InPos states for X,Y,Z,A,B axes.
 * Bit 0 = X, Bit 1 = Y, Bit 2 = Z, Bit 3 = A, Bit 4 = B.
 *
 * @param[out] inpos_bitmap Pointer to store 5-bit InPos bitmap
 *
 * @return esp_err_t
 *         - ESP_OK: Read successful
 *         - ESP_ERR_INVALID_STATE: Not initialized
 *         - ESP_ERR_INVALID_ARG: NULL pointer
 *         - ESP_ERR_TIMEOUT: I2C read timeout
 */
esp_err_t servo_feedback_get_all_inpos(uint8_t *inpos_bitmap);

/**
 * @brief Deinitialize servo feedback reader
 *
 * @return esp_err_t
 *         - ESP_OK: Deinitialized successfully
 */
esp_err_t servo_feedback_deinit(void);

/** @} */ // end servo_feedback

#ifdef __cplusplus
}
#endif

#endif // SERVO_FEEDBACK_H
