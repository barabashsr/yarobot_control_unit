/**
 * @file response_formatter.h
 * @brief Response formatter public API for generating USB responses
 * @author YaRobot Team
 * @date 2025
 *
 * @note Formats responses, errors, and events according to protocol spec.
 *       Thread-safe: all functions operate on caller-provided buffers.
 */

#ifndef RESPONSE_FORMATTER_H
#define RESPONSE_FORMATTER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup response_formatter Response Formatter
 * @brief Generate protocol-compliant response strings
 * @{
 */

/**
 * @defgroup event_types Event Types
 * @brief Asynchronous event type enumeration
 * @{
 */

/**
 * @brief Event type enumeration
 *
 * Matches event types defined in protocol specification.
 */
typedef enum {
    /** @brief Motion completed successfully */
    EVTTYPE_MOTION_COMPLETE,
    /** @brief Motion error occurred */
    EVTTYPE_MOTION_ERROR,
    /** @brief Limit switch triggered */
    EVTTYPE_LIMIT_TRIGGERED,
    /** @brief Emergency stop state changed */
    EVTTYPE_ESTOP_CHANGED,
    /** @brief Operating mode changed */
    EVTTYPE_MODE_CHANGED,
    /** @brief Generic error event */
    EVTTYPE_ERROR,
    /** @brief Width measurement complete */
    EVTTYPE_WIDTH_MEASURED,
    /** @brief System boot event */
    EVTTYPE_BOOT,
} EventType;

/** @} */ // end event_types

/**
 * @defgroup event_struct Event Structure
 * @brief Asynchronous event data structure
 * @{
 */

/**
 * @brief Limit switch state
 */
typedef enum {
    /** @brief Minimum limit triggered */
    LIMIT_STATE_MIN = 0,
    /** @brief Maximum limit triggered */
    LIMIT_STATE_MAX = 1,
} LimitState;

/**
 * @brief Event data structure
 *
 * Contains all information needed to format an event notification.
 * The axis field uses 0xFF for system-wide events.
 */
typedef struct {
    /** @brief Event type */
    EventType type;

    /** @brief Axis index (0-7) or 0xFF for system-wide events */
    uint8_t axis;

    /** @brief Event-specific data */
    union {
        /** @brief Position value for motion complete events */
        float position;
        /** @brief Width value for width measurement events */
        float width;
        /** @brief Error code for error events */
        uint8_t error_code;
        /** @brief Limit switch state (MIN=0, MAX=1) */
        uint8_t limit_state;
        /** @brief E-stop active state */
        bool estop_active;
    } data;

    /** @brief Event timestamp (microseconds since boot) */
    int64_t timestamp;
} Event;

/** @} */ // end event_struct

/**
 * @brief Format simple OK response
 *
 * Generates "OK\r\n" in the provided buffer.
 *
 * @param[out] buf Buffer to write response into
 * @param[in] len Size of buffer in bytes
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if buf is NULL
 * @return ESP_ERR_INVALID_SIZE if buffer too small
 *
 * @note Thread-safe: operates on caller-provided buffer only.
 */
esp_err_t format_ok(char* buf, size_t len);

/**
 * @brief Format OK response with data
 *
 * Generates "OK <formatted_data>\r\n" in the provided buffer.
 * Uses printf-style format string and variable arguments.
 *
 * @param[out] buf Buffer to write response into
 * @param[in] len Size of buffer in bytes
 * @param[in] fmt Printf-style format string
 * @param[in] ... Format arguments
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if buf or fmt is NULL
 * @return ESP_ERR_INVALID_SIZE if buffer too small or output truncated
 *
 * @note Thread-safe: operates on caller-provided buffer only.
 *
 * @par Example
 * @code
 * char buf[64];
 * format_ok_data(buf, sizeof(buf), "X %.3f", 123.456);
 * // Result: "OK X 123.456\r\n"
 * @endcode
 */
esp_err_t format_ok_data(char* buf, size_t len, const char* fmt, ...);

/**
 * @brief Format error response
 *
 * Generates "ERROR <code> <message>\r\n" in the provided buffer.
 *
 * @param[out] buf Buffer to write response into
 * @param[in] len Size of buffer in bytes
 * @param[in] code Error code string (e.g., ERR_INVALID_COMMAND)
 * @param[in] msg Error message string (e.g., MSG_INVALID_COMMAND)
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if buf, code, or msg is NULL
 * @return ESP_ERR_INVALID_SIZE if buffer too small
 *
 * @note Thread-safe: operates on caller-provided buffer only.
 *
 * @par Example
 * @code
 * char buf[64];
 * format_error(buf, sizeof(buf), ERR_INVALID_COMMAND, MSG_INVALID_COMMAND);
 * // Result: "ERROR E001 Invalid command\r\n"
 * @endcode
 */
esp_err_t format_error(char* buf, size_t len, const char* code, const char* msg);

/**
 * @brief Format event notification
 *
 * Generates "EVENT <type> <axis> [data]\r\n" in the provided buffer.
 * Format depends on event type.
 *
 * @param[out] buf Buffer to write response into
 * @param[in] len Size of buffer in bytes
 * @param[in] event Pointer to Event structure
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if buf or event is NULL
 * @return ESP_ERR_INVALID_SIZE if buffer too small
 *
 * @note Thread-safe: operates on caller-provided buffer only.
 *
 * @par Examples
 * @code
 * Event evt = {.type = EVTTYPE_MOTION_COMPLETE, .axis = 0, .data.position = 100.0f};
 * format_event(buf, sizeof(buf), &evt);
 * // Result: "EVENT DONE X 100.000\r\n"
 *
 * Event limit_evt = {.type = EVTTYPE_LIMIT_TRIGGERED, .axis = 1, .data.limit_state = 0};
 * format_event(buf, sizeof(buf), &limit_evt);
 * // Result: "EVENT LIMIT Y MIN\r\n"
 *
 * Event estop_evt = {.type = EVTTYPE_ESTOP_CHANGED, .data.estop_active = true};
 * format_event(buf, sizeof(buf), &estop_evt);
 * // Result: "EVENT ESTOP ACTIVE\r\n"
 * @endcode
 */
esp_err_t format_event(char* buf, size_t len, const Event* event);

/** @} */ // end response_formatter

#ifdef __cplusplus
}
#endif

#endif // RESPONSE_FORMATTER_H
