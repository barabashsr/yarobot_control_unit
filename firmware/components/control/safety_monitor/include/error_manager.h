/**
 * @file error_manager.h
 * @brief Error Manager API - Tracks error counts and axis error states
 * @author YaRobot Team
 * @date 2025
 *
 * @note Story 4-10: Error Tracking & Recovery
 *       - Tracks cumulative error counts per category (I2C, TIMEOUT, LIMIT, ESTOP, POSLOS)
 *       - Tracks per-axis error counts
 *       - Manages axis ERROR state (blocks motion until cleared)
 *       - Publishes ERROR events with context
 *       - Thread-safe access via FreeRTOS mutex
 *
 * @see config_commands.h for CMD_ERRCNT, CMD_CLRERR, ErrorCategory, error codes
 * @see config_limits.h for LIMIT_ERROR_CATEGORIES, LIMIT_NUM_AXES
 */

#ifndef ERROR_MANAGER_H
#define ERROR_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "config_commands.h"
#include "config_limits.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup error_manager Error Manager
 * @brief Error tracking and axis error state management
 * @{
 */

/**
 * @brief Maximum error code string length (e.g., "E041")
 */
#define ERROR_CODE_MAX_LEN  8

/**
 * @brief Category name strings for ERRCNT response formatting
 */
extern const char* const ERROR_CATEGORY_NAMES[];

/**
 * @defgroup error_manager_api Error Manager API
 * @brief Initialization and error management functions
 * @{
 */

/**
 * @brief Initialize error manager
 *
 * Performs the following:
 * - Zeroes all global error counters
 * - Zeroes all per-axis error counters
 * - Clears all axis ERROR states
 * - Creates mutex for thread-safe access
 *
 * @return esp_err_t
 *         - ESP_OK: Initialized successfully
 *         - ESP_ERR_INVALID_STATE: Already initialized
 *         - ESP_ERR_NO_MEM: Failed to create mutex
 */
esp_err_t error_manager_init(void);

/**
 * @brief Check if error manager is initialized
 *
 * @return true if initialized, false otherwise
 */
bool error_manager_is_initialized(void);

/**
 * @brief Deinitialize error manager
 *
 * Resets all state and frees mutex. Safe to call if not initialized.
 *
 * @return esp_err_t
 *         - ESP_OK: Deinitialized successfully
 */
esp_err_t error_manager_deinit(void);

/** @} */ // end error_manager_api

/**
 * @defgroup error_counts Error Count Functions
 * @brief Functions for tracking and retrieving error counts (AC1, AC2, AC7)
 * @{
 */

/**
 * @brief Increment error counter for a category
 *
 * Increments both global counter and per-axis counter (if axis specified).
 * Optionally publishes an ERROR event.
 *
 * @param[in] category Error category (ERR_CAT_I2C, etc.)
 * @param[in] axis Axis index (0-7) or 0xFF for system-wide errors
 * @param[in] publish_event true to publish EVENT ERROR
 *
 * @return esp_err_t
 *         - ESP_OK: Counter incremented
 *         - ESP_ERR_INVALID_ARG: Invalid category
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t error_manager_increment(ErrorCategory category, uint8_t axis, bool publish_event);

/**
 * @brief Get global error counts for all categories
 *
 * @param[out] counts Array of size ERR_CAT_COUNT to receive counts
 *
 * @return esp_err_t
 *         - ESP_OK: Counts retrieved
 *         - ESP_ERR_INVALID_ARG: counts is NULL
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t error_manager_get_counts(uint32_t* counts);

/**
 * @brief Get error counts for a specific axis
 *
 * @param[in] axis Axis index (0-7)
 * @param[out] counts Array of size ERR_CAT_COUNT to receive per-axis counts
 *
 * @return esp_err_t
 *         - ESP_OK: Counts retrieved
 *         - ESP_ERR_INVALID_ARG: Invalid axis or counts is NULL
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t error_manager_get_axis_counts(uint8_t axis, uint32_t* counts);

/**
 * @brief Clear all error counters (global and per-axis)
 *
 * @return esp_err_t
 *         - ESP_OK: All counters cleared
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t error_manager_clear_counts(void);

/** @} */ // end error_counts

/**
 * @defgroup error_state Axis Error State Functions
 * @brief Functions for managing axis ERROR state (AC4, AC5, AC6)
 * @{
 */

/**
 * @brief Set axis into ERROR state
 *
 * Axis in ERROR state will block motion commands until cleared.
 *
 * @param[in] axis Axis index (0-7)
 * @param[in] error_code Error code string (e.g., ERR_LIMIT_ACTIVE)
 *
 * @return esp_err_t
 *         - ESP_OK: Axis set to ERROR state
 *         - ESP_ERR_INVALID_ARG: Invalid axis or error_code is NULL
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t error_manager_set_axis_error(uint8_t axis, const char* error_code);

/**
 * @brief Clear ERROR state for an axis
 *
 * Returns axis to normal state, allowing motion commands.
 *
 * @param[in] axis Axis index (0-7)
 *
 * @return esp_err_t
 *         - ESP_OK: ERROR state cleared
 *         - ESP_ERR_INVALID_ARG: Invalid axis
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t error_manager_clear_axis_error(uint8_t axis);

/**
 * @brief Clear ERROR state for all axes
 *
 * @return esp_err_t
 *         - ESP_OK: All ERROR states cleared
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t error_manager_clear_all_axis_errors(void);

/**
 * @brief Check if axis is in ERROR state
 *
 * @param[in] axis Axis index (0-7)
 *
 * @return true if axis is in ERROR state
 * @return false if axis is not in error or axis is invalid
 */
bool error_manager_is_axis_in_error(uint8_t axis);

/**
 * @brief Get error code for an axis in ERROR state
 *
 * @param[in] axis Axis index (0-7)
 * @param[out] error_code Buffer to receive error code (min ERROR_CODE_MAX_LEN bytes)
 * @param[in] buf_len Size of error_code buffer
 *
 * @return esp_err_t
 *         - ESP_OK: Error code retrieved
 *         - ESP_ERR_NOT_FOUND: Axis not in ERROR state
 *         - ESP_ERR_INVALID_ARG: Invalid axis or buffer
 *         - ESP_ERR_INVALID_STATE: Not initialized
 */
esp_err_t error_manager_get_axis_error_code(uint8_t axis, char* error_code, size_t buf_len);

/** @} */ // end error_state

/**
 * @defgroup error_events Error Event Functions
 * @brief Functions for publishing error events (AC3)
 * @{
 */

/**
 * @brief Publish an ERROR event
 *
 * Formats and publishes: "EVENT ERROR <axis|SYSTEM> <code>"
 *
 * @param[in] category Error category for logging
 * @param[in] axis Axis index (0-7) or 0xFF for SYSTEM
 * @param[in] error_code Error code string (e.g., ERR_I2C_FAILURE)
 *
 * @return esp_err_t
 *         - ESP_OK: Event published
 *         - ESP_ERR_INVALID_ARG: Invalid error_code
 */
esp_err_t error_manager_publish_error(ErrorCategory category, uint8_t axis, const char* error_code);

/** @} */ // end error_events

/**
 * @defgroup error_test Test/Debug Functions
 * @brief Functions for testing error manager
 * @{
 */

/**
 * @brief Inject error count for testing
 *
 * @param[in] category Error category
 * @param[in] count Count to set
 */
void error_manager_inject_test_count(ErrorCategory category, uint32_t count);

/**
 * @brief Inject axis error state for testing
 *
 * @param[in] axis Axis index (0-7)
 * @param[in] in_error true to set error, false to clear
 * @param[in] error_code Error code (if in_error is true)
 */
void error_manager_inject_test_error(uint8_t axis, bool in_error, const char* error_code);

/** @} */ // end error_test

/** @} */ // end error_manager

#ifdef __cplusplus
}
#endif

#endif // ERROR_MANAGER_H
