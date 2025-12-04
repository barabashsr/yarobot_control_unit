/**
 * @file response_formatter.c
 * @brief Response formatter implementation
 * @author YaRobot Team
 * @date 2025
 *
 * @note All format functions operate on caller-provided buffers,
 *       ensuring thread safety for concurrent access.
 */

#include "response_formatter.h"
#include "command_parser.h"
#include "config.h"
#include "config_commands.h"
#include "config_limits.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/** @brief Response line terminator (CR+LF) */
#define RESP_TERMINATOR "\r\n"

/** @brief Length of response terminator */
#define RESP_TERMINATOR_LEN 2

esp_err_t format_ok(char* buf, size_t len)
{
    if (buf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Calculate required length: "OK" + "\r\n" + null terminator
    size_t required = strlen(RESP_OK) + RESP_TERMINATOR_LEN + 1;

    if (len < required) {
        return ESP_ERR_INVALID_SIZE;
    }

    int written = snprintf(buf, len, "%s" RESP_TERMINATOR, RESP_OK);

    if (written < 0 || (size_t)written >= len) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

esp_err_t format_ok_data(char* buf, size_t len, const char* fmt, ...)
{
    if (buf == NULL || fmt == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // First, write "OK " prefix
    size_t prefix_len = strlen(RESP_OK) + 1;  // "OK "
    if (len <= prefix_len + RESP_TERMINATOR_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    int offset = snprintf(buf, len, "%s ", RESP_OK);
    if (offset < 0 || (size_t)offset >= len) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Format the data portion
    va_list args;
    va_start(args, fmt);
    int data_written = vsnprintf(buf + offset, len - offset, fmt, args);
    va_end(args);

    if (data_written < 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    offset += data_written;

    // Check if we have room for terminator
    if ((size_t)offset + RESP_TERMINATOR_LEN >= len) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Append terminator
    memcpy(buf + offset, RESP_TERMINATOR, RESP_TERMINATOR_LEN + 1);

    return ESP_OK;
}

esp_err_t format_error(char* buf, size_t len, const char* code, const char* msg)
{
    if (buf == NULL || code == NULL || msg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int written = snprintf(buf, len, "%s %s %s" RESP_TERMINATOR,
                           RESP_ERROR, code, msg);

    if (written < 0 || (size_t)written >= len) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

esp_err_t format_event(char* buf, size_t len, const Event* event)
{
    if (buf == NULL || event == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int written = 0;
    char axis_char;

    switch (event->type) {
        case EVTTYPE_MOTION_COMPLETE:
            axis_char = index_to_axis(event->axis);
            if (axis_char == '\0') {
                return ESP_ERR_INVALID_ARG;
            }
            written = snprintf(buf, len, "%s %s %c %.3f" RESP_TERMINATOR,
                               RESP_EVENT, EVT_MOTION_COMPLETE,
                               axis_char, event->data.position);
            break;

        case EVTTYPE_LIMIT_TRIGGERED:
            axis_char = index_to_axis(event->axis);
            if (axis_char == '\0') {
                return ESP_ERR_INVALID_ARG;
            }
            written = snprintf(buf, len, "%s %s %c %s" RESP_TERMINATOR,
                               RESP_EVENT, EVT_LIMIT_TRIGGERED,
                               axis_char,
                               (event->data.limit_state == LIMIT_STATE_MIN) ? "MIN" : "MAX");
            break;

        case EVTTYPE_ESTOP_CHANGED:
            written = snprintf(buf, len, "%s %s %s" RESP_TERMINATOR,
                               RESP_EVENT, EVT_ESTOP_ACTIVATED,
                               event->data.estop_active ? "ACTIVE" : "RELEASED");
            break;

        case EVTTYPE_BOOT:
            // Format: EVENT BOOT V<version> AXES:<count> STATE:IDLE
            written = snprintf(buf, len, "%s %s V%s AXES:%d STATE:IDLE" RESP_TERMINATOR,
                               RESP_EVENT, EVT_BOOT, FIRMWARE_VERSION_STRING, LIMIT_NUM_AXES);
            break;

        case EVTTYPE_MODE_CHANGED:
            // Format: EVENT MODE <mode_name>
            // Mode name is stored as string pointer in data.mode_name
            if (event->data.mode_name == NULL) {
                return ESP_ERR_INVALID_ARG;
            }
            written = snprintf(buf, len, "%s %s %s" RESP_TERMINATOR,
                               RESP_EVENT, EVT_MODE, event->data.mode_name);
            break;

        case EVTTYPE_MOTION_ERROR:
            axis_char = index_to_axis(event->axis);
            if (axis_char == '\0') {
                return ESP_ERR_INVALID_ARG;
            }
            written = snprintf(buf, len, "%s ERROR %c E%03d" RESP_TERMINATOR,
                               RESP_EVENT, axis_char, event->data.error_code);
            break;

        case EVTTYPE_ERROR:
            written = snprintf(buf, len, "%s ERROR E%03d" RESP_TERMINATOR,
                               RESP_EVENT, event->data.error_code);
            break;

        case EVTTYPE_WIDTH_MEASURED:
            axis_char = index_to_axis(event->axis);
            if (axis_char == '\0') {
                return ESP_ERR_INVALID_ARG;
            }
            written = snprintf(buf, len, "%s WIDTH %c %.3f" RESP_TERMINATOR,
                               RESP_EVENT, axis_char, event->data.width);
            break;

        default:
            return ESP_ERR_INVALID_ARG;
    }

    if (written < 0 || (size_t)written >= len) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}
