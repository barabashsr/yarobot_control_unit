/**
 * @file test_response_formatter.c
 * @brief Unit tests for response formatter component
 * @author YaRobot Team
 * @date 2025
 *
 * Tests all acceptance criteria from Story 2.3
 */

#include "unity.h"
#include "response_formatter.h"
#include "config_commands.h"
#include "config_limits.h"
#include <string.h>

/**
 * AC4: Given format_ok() is called, when generating simple acknowledgment,
 * then response is exactly "OK\r\n"
 */
TEST_CASE("AC4: format_ok generates OK\\r\\n", "[response_formatter]")
{
    char buf[64];
    TEST_ASSERT_EQUAL(ESP_OK, format_ok(buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_STRING("OK\r\n", buf);
}

/**
 * AC5: Given format_ok_data() is called with position data,
 * when generating position response, then format is "OK X 123.456\r\n"
 */
TEST_CASE("AC5: format_ok_data with position", "[response_formatter]")
{
    char buf[64];
    TEST_ASSERT_EQUAL(ESP_OK, format_ok_data(buf, sizeof(buf), "X %.3f", 123.456f));
    TEST_ASSERT_EQUAL_STRING("OK X 123.456\r\n", buf);
}

/**
 * AC1: Given command processing completes successfully,
 * when a response is generated, then it follows the format "OK [data]\r\n"
 */
TEST_CASE("AC1: format_ok_data various formats", "[response_formatter]")
{
    char buf[64];

    TEST_ASSERT_EQUAL(ESP_OK, format_ok_data(buf, sizeof(buf), "READY"));
    TEST_ASSERT_EQUAL_STRING("OK READY\r\n", buf);

    TEST_ASSERT_EQUAL(ESP_OK, format_ok_data(buf, sizeof(buf), "Y %.3f", -50.5f));
    TEST_ASSERT_EQUAL_STRING("OK Y -50.500\r\n", buf);

    TEST_ASSERT_EQUAL(ESP_OK, format_ok_data(buf, sizeof(buf), "AXES:%d", 8));
    TEST_ASSERT_EQUAL_STRING("OK AXES:8\r\n", buf);
}

/**
 * AC6: Given format_error() is called with ERR_INVALID_COMMAND,
 * when generating error, then response is "ERROR E001 Invalid command\r\n"
 */
TEST_CASE("AC6: format_error with ERR_INVALID_COMMAND", "[response_formatter]")
{
    char buf[64];
    TEST_ASSERT_EQUAL(ESP_OK, format_error(buf, sizeof(buf),
                                           ERR_INVALID_COMMAND, MSG_INVALID_COMMAND));
    TEST_ASSERT_EQUAL_STRING("ERROR E001 Invalid command\r\n", buf);
}

/**
 * AC2: Given command processing fails,
 * when an error response is generated, then it follows the format "ERROR <code> <message>\r\n"
 */
TEST_CASE("AC2: format_error with various error codes", "[response_formatter]")
{
    char buf[64];

    TEST_ASSERT_EQUAL(ESP_OK, format_error(buf, sizeof(buf),
                                           ERR_INVALID_AXIS, MSG_INVALID_AXIS));
    TEST_ASSERT_EQUAL_STRING("ERROR E002 Invalid axis\r\n", buf);

    TEST_ASSERT_EQUAL(ESP_OK, format_error(buf, sizeof(buf),
                                           ERR_INVALID_PARAMETER, MSG_INVALID_PARAMETER));
    TEST_ASSERT_EQUAL_STRING("ERROR E003 Invalid parameter\r\n", buf);

    TEST_ASSERT_EQUAL(ESP_OK, format_error(buf, sizeof(buf),
                                           ERR_EMERGENCY_STOP, MSG_EMERGENCY_STOP));
    TEST_ASSERT_EQUAL_STRING("ERROR E006 Emergency stop active\r\n", buf);
}

/**
 * AC7: Given format_event() is called for motion complete,
 * when generating event, then format is "EVENT DONE X 100.000\r\n"
 */
TEST_CASE("AC7: format_event motion complete", "[response_formatter]")
{
    char buf[64];
    Event evt = {
        .type = EVTTYPE_MOTION_COMPLETE,
        .axis = 0,  // X
        .data.position = 100.0f,
        .timestamp = 0
    };
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    TEST_ASSERT_EQUAL_STRING("EVENT DONE X 100.000\r\n", buf);
}

/**
 * AC8: Given format_event() is called for limit trigger,
 * when generating event, then format is "EVENT LIMIT Y MIN\r\n"
 */
TEST_CASE("AC8: format_event limit triggered MIN", "[response_formatter]")
{
    char buf[64];
    Event evt = {
        .type = EVTTYPE_LIMIT_TRIGGERED,
        .axis = 1,  // Y
        .data.limit_state = LIMIT_STATE_MIN,
        .timestamp = 0
    };
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    TEST_ASSERT_EQUAL_STRING("EVENT LIMIT Y MIN\r\n", buf);
}

/**
 * Additional test for limit MAX
 */
TEST_CASE("AC8: format_event limit triggered MAX", "[response_formatter]")
{
    char buf[64];
    Event evt = {
        .type = EVTTYPE_LIMIT_TRIGGERED,
        .axis = 2,  // Z
        .data.limit_state = LIMIT_STATE_MAX,
        .timestamp = 0
    };
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    TEST_ASSERT_EQUAL_STRING("EVENT LIMIT Z MAX\r\n", buf);
}

/**
 * AC9: Given format_event() is called for E-stop,
 * when generating event, then format is "EVENT ESTOP ACTIVE\r\n"
 */
TEST_CASE("AC9: format_event estop active", "[response_formatter]")
{
    char buf[64];
    Event evt = {
        .type = EVTTYPE_ESTOP_CHANGED,
        .axis = 0xFF,  // system-wide
        .data.estop_active = true,
        .timestamp = 0
    };
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    TEST_ASSERT_EQUAL_STRING("EVENT ESTOP ACTIVE\r\n", buf);
}

/**
 * Additional test for E-stop released
 */
TEST_CASE("AC9: format_event estop released", "[response_formatter]")
{
    char buf[64];
    Event evt = {
        .type = EVTTYPE_ESTOP_CHANGED,
        .axis = 0xFF,
        .data.estop_active = false,
        .timestamp = 0
    };
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    TEST_ASSERT_EQUAL_STRING("EVENT ESTOP RELEASED\r\n", buf);
}

/**
 * AC3: Given an asynchronous event occurs,
 * when an event notification is generated, then it follows the format "EVENT <type> <axis> [data]\r\n"
 */
TEST_CASE("AC3: format_event boot", "[response_formatter]")
{
    char buf[64];
    Event evt = {
        .type = EVTTYPE_BOOT,
        .axis = 0xFF,
        .timestamp = 0
    };
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    TEST_ASSERT_EQUAL_STRING("EVENT BOOT V1.0.0 AXES:8 STATE:IDLE\r\n", buf);
}

/**
 * Test format_event mode changed
 */
TEST_CASE("format_event mode changed", "[response_formatter]")
{
    char buf[64];
    Event evt = {
        .type = EVTTYPE_MODE_CHANGED,
        .axis = 0xFF,
        .data.error_code = 1,  // READY mode
        .timestamp = 0
    };
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    TEST_ASSERT_EQUAL_STRING("EVENT MODE READY\r\n", buf);
}

/**
 * AC10: Given any response, when terminator is added, then it is exactly "\r\n" (CR+LF)
 */
TEST_CASE("AC10: All outputs terminate with CRLF", "[response_formatter]")
{
    char buf[64];
    size_t len;

    TEST_ASSERT_EQUAL(ESP_OK, format_ok(buf, sizeof(buf)));
    len = strlen(buf);
    TEST_ASSERT_GREATER_OR_EQUAL(2, len);
    TEST_ASSERT_EQUAL('\r', buf[len - 2]);
    TEST_ASSERT_EQUAL('\n', buf[len - 1]);

    TEST_ASSERT_EQUAL(ESP_OK, format_ok_data(buf, sizeof(buf), "TEST"));
    len = strlen(buf);
    TEST_ASSERT_EQUAL('\r', buf[len - 2]);
    TEST_ASSERT_EQUAL('\n', buf[len - 1]);

    TEST_ASSERT_EQUAL(ESP_OK, format_error(buf, sizeof(buf), "E001", "Test"));
    len = strlen(buf);
    TEST_ASSERT_EQUAL('\r', buf[len - 2]);
    TEST_ASSERT_EQUAL('\n', buf[len - 1]);

    Event evt = {.type = EVTTYPE_MOTION_COMPLETE, .axis = 0, .data.position = 0.0f};
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    len = strlen(buf);
    TEST_ASSERT_EQUAL('\r', buf[len - 2]);
    TEST_ASSERT_EQUAL('\n', buf[len - 1]);
}

/**
 * AC11: Given response buffer size, when formatting,
 * then output never exceeds LIMIT_RESPONSE_MAX_LENGTH
 */
TEST_CASE("AC11: Buffer overflow protection format_ok", "[response_formatter]")
{
    char tiny_buf[3];  // Too small for "OK\r\n" (needs 5 bytes including null)
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, format_ok(tiny_buf, sizeof(tiny_buf)));
}

TEST_CASE("AC11: Buffer overflow protection format_ok_data", "[response_formatter]")
{
    char tiny_buf[8];  // Too small for "OK very long data\r\n"
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, format_ok_data(tiny_buf, sizeof(tiny_buf),
                                                           "very long data"));
}

TEST_CASE("AC11: Buffer overflow protection format_error", "[response_formatter]")
{
    char tiny_buf[10];  // Too small for "ERROR E001 message\r\n"
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, format_error(tiny_buf, sizeof(tiny_buf),
                                                          ERR_INVALID_COMMAND, MSG_INVALID_COMMAND));
}

TEST_CASE("AC11: Buffer overflow protection format_event", "[response_formatter]")
{
    char tiny_buf[10];  // Too small for event
    Event evt = {.type = EVTTYPE_MOTION_COMPLETE, .axis = 0, .data.position = 100.0f};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, format_event(tiny_buf, sizeof(tiny_buf), &evt));
}

/**
 * AC11: Verify output fits within LIMIT_RESPONSE_MAX_LENGTH
 */
TEST_CASE("AC11: Output within max length", "[response_formatter]")
{
    char buf[LIMIT_RESPONSE_MAX_LENGTH];

    TEST_ASSERT_EQUAL(ESP_OK, format_ok(buf, sizeof(buf)));
    TEST_ASSERT_LESS_OR_EQUAL(LIMIT_RESPONSE_MAX_LENGTH, strlen(buf) + 1);

    TEST_ASSERT_EQUAL(ESP_OK, format_ok_data(buf, sizeof(buf), "X %.3f", 999999.999f));
    TEST_ASSERT_LESS_OR_EQUAL(LIMIT_RESPONSE_MAX_LENGTH, strlen(buf) + 1);

    TEST_ASSERT_EQUAL(ESP_OK, format_error(buf, sizeof(buf),
                                           ERR_INVALID_COMMAND, MSG_INVALID_COMMAND));
    TEST_ASSERT_LESS_OR_EQUAL(LIMIT_RESPONSE_MAX_LENGTH, strlen(buf) + 1);

    Event evt = {.type = EVTTYPE_BOOT};
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    TEST_ASSERT_LESS_OR_EQUAL(LIMIT_RESPONSE_MAX_LENGTH, strlen(buf) + 1);
}

/**
 * AC14: All response/error/event string literals use constants from config_commands.h
 * This is verified by the implementation using RESP_OK, RESP_ERROR, RESP_EVENT, etc.
 * We verify the output matches expected values that come from those constants.
 */
TEST_CASE("AC14: Uses config constants", "[response_formatter]")
{
    char buf[64];

    // Verify RESP_OK is used
    TEST_ASSERT_EQUAL(ESP_OK, format_ok(buf, sizeof(buf)));
    TEST_ASSERT_NOT_NULL(strstr(buf, RESP_OK));

    // Verify RESP_ERROR is used
    TEST_ASSERT_EQUAL(ESP_OK, format_error(buf, sizeof(buf), "E999", "Test"));
    TEST_ASSERT_NOT_NULL(strstr(buf, RESP_ERROR));

    // Verify RESP_EVENT is used
    Event evt = {.type = EVTTYPE_MOTION_COMPLETE, .axis = 0, .data.position = 0.0f};
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    TEST_ASSERT_NOT_NULL(strstr(buf, RESP_EVENT));

    // Verify EVT_MOTION_COMPLETE is used
    TEST_ASSERT_NOT_NULL(strstr(buf, EVT_MOTION_COMPLETE));

    // Verify EVT_LIMIT_TRIGGERED is used
    evt.type = EVTTYPE_LIMIT_TRIGGERED;
    evt.axis = 0;
    evt.data.limit_state = LIMIT_STATE_MIN;
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    TEST_ASSERT_NOT_NULL(strstr(buf, EVT_LIMIT_TRIGGERED));

    // Verify EVT_ESTOP_ACTIVATED is used
    evt.type = EVTTYPE_ESTOP_CHANGED;
    evt.data.estop_active = true;
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    TEST_ASSERT_NOT_NULL(strstr(buf, EVT_ESTOP_ACTIVATED));

    // Verify EVT_BOOT is used
    evt.type = EVTTYPE_BOOT;
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    TEST_ASSERT_NOT_NULL(strstr(buf, EVT_BOOT));
}

/**
 * NULL argument handling
 */
TEST_CASE("NULL argument handling", "[response_formatter]")
{
    char buf[64];

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, format_ok(NULL, 64));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, format_ok_data(NULL, 64, "test"));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, format_ok_data(buf, sizeof(buf), NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, format_error(NULL, 64, "E001", "test"));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, format_error(buf, sizeof(buf), NULL, "test"));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, format_error(buf, sizeof(buf), "E001", NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, format_event(NULL, 64, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, format_event(buf, sizeof(buf), NULL));
}

/**
 * Invalid axis in event
 */
TEST_CASE("Invalid axis in event", "[response_formatter]")
{
    char buf[64];

    // Invalid axis index for motion complete
    Event evt = {
        .type = EVTTYPE_MOTION_COMPLETE,
        .axis = 99,  // Invalid
        .data.position = 100.0f
    };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, format_event(buf, sizeof(buf), &evt));
}

/**
 * Test all axis letters
 */
TEST_CASE("format_event with all axes", "[response_formatter]")
{
    char buf[64];
    const char* expected_axes[] = {"X", "Y", "Z", "A", "B", "C", "D", "E"};

    for (uint8_t i = 0; i < 8; i++) {
        Event evt = {
            .type = EVTTYPE_MOTION_COMPLETE,
            .axis = i,
            .data.position = (float)i * 10.0f
        };
        TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
        TEST_ASSERT_NOT_NULL(strstr(buf, expected_axes[i]));
    }
}

/**
 * Test width measured event
 */
TEST_CASE("format_event width measured", "[response_formatter]")
{
    char buf[64];
    Event evt = {
        .type = EVTTYPE_WIDTH_MEASURED,
        .axis = 0,  // X
        .data.width = 456.789f,
        .timestamp = 0
    };
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    TEST_ASSERT_EQUAL_STRING("EVENT WIDTH X 456.789\r\n", buf);
}

/**
 * Test motion error event
 */
TEST_CASE("format_event motion error", "[response_formatter]")
{
    char buf[64];
    Event evt = {
        .type = EVTTYPE_MOTION_ERROR,
        .axis = 3,  // A
        .data.error_code = 5,
        .timestamp = 0
    };
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    TEST_ASSERT_EQUAL_STRING("EVENT ERROR A E005\r\n", buf);
}

/**
 * Test generic error event
 */
TEST_CASE("format_event generic error", "[response_formatter]")
{
    char buf[64];
    Event evt = {
        .type = EVTTYPE_ERROR,
        .axis = 0xFF,
        .data.error_code = 11,
        .timestamp = 0
    };
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    TEST_ASSERT_EQUAL_STRING("EVENT ERROR E011\r\n", buf);
}

/**
 * AC12: Thread safety is ensured by design - all functions use caller-provided buffers.
 * This test verifies no static state corruption with multiple calls.
 */
TEST_CASE("AC12: Thread safety - no static state", "[response_formatter]")
{
    char buf1[64], buf2[64], buf3[64];

    // Rapidly format different responses to verify no cross-contamination
    for (int i = 0; i < 100; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, format_ok(buf1, sizeof(buf1)));
        TEST_ASSERT_EQUAL(ESP_OK, format_ok_data(buf2, sizeof(buf2), "X %.3f", 1.0f * i));
        TEST_ASSERT_EQUAL(ESP_OK, format_error(buf3, sizeof(buf3),
                                               ERR_INVALID_COMMAND, MSG_INVALID_COMMAND));

        // Verify buf1 is always exactly "OK\r\n"
        TEST_ASSERT_EQUAL_STRING("OK\r\n", buf1);

        // Verify buf3 is always the error format
        TEST_ASSERT_EQUAL_STRING("ERROR E001 Invalid command\r\n", buf3);
    }
}

/**
 * Test exact minimum buffer sizes
 */
TEST_CASE("Exact minimum buffer sizes", "[response_formatter]")
{
    char buf[5];  // Exactly "OK\r\n" + null = 5 bytes

    TEST_ASSERT_EQUAL(ESP_OK, format_ok(buf, 5));
    TEST_ASSERT_EQUAL_STRING("OK\r\n", buf);

    // One byte less should fail
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, format_ok(buf, 4));
}

/**
 * Test negative position values
 */
TEST_CASE("Negative position in event", "[response_formatter]")
{
    char buf[64];
    Event evt = {
        .type = EVTTYPE_MOTION_COMPLETE,
        .axis = 0,
        .data.position = -123.456f
    };
    TEST_ASSERT_EQUAL(ESP_OK, format_event(buf, sizeof(buf), &evt));
    TEST_ASSERT_EQUAL_STRING("EVENT DONE X -123.456\r\n", buf);
}
