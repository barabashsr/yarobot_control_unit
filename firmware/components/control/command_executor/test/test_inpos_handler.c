/**
 * @file test_inpos_handler.c
 * @brief Integration tests for INPOS command handler
 * @author YaRobot Team
 * @date 2025
 *
 * Tests acceptance criteria from Story 4-8:
 *   AC2: INPOS (no param) returns all 5 servo axes: "OK X:1 Y:0 Z:1 A:1 B:0"
 *   AC3: INPOS X returns single axis: "OK X 1"
 *   AC5: INPOS C returns N/A for stepper: "OK C N/A"
 *   AC6: INPOS Q returns error for invalid axis
 *
 * @note Tests require command_executor and servo_feedback initialized.
 */

#include "unity.h"
#include "inpos_handler.h"
#include "servo_feedback.h"
#include "command_executor.h"
#include "command_parser.h"
#include "config_commands.h"
#include "config_axes.h"
#include "mcp23017_wrapper.h"

#include <string.h>
#include "esp_log.h"

static const char* TAG = "test_inpos";

/* ==========================================================================
 * Test Helpers
 * ========================================================================== */

/**
 * @brief Create parsed command for INPOS
 */
static void make_inpos_command(ParsedCommand* cmd, char axis)
{
    memset(cmd, 0, sizeof(*cmd));
    strncpy(cmd->verb, CMD_INPOS, sizeof(cmd->verb) - 1);
    cmd->axis = axis;
    cmd->param_count = 0;
    cmd->has_str_param = false;
}

/**
 * @brief Check if test prerequisites are met
 */
static bool ensure_test_ready(void)
{
    /* MCP23017 must be initialized for servo_feedback to work */
    if (!mcp23017_wrapper_is_initialized()) {
        return false;
    }

    /* Initialize servo_feedback if needed */
    if (!servo_feedback_is_initialized()) {
        if (servo_feedback_init() != ESP_OK) {
            return false;
        }
    }

    return true;
}

/* ==========================================================================
 * AC2: INPOS without parameter returns all axes
 * ========================================================================== */

/**
 * AC2: INPOS returns all 5 servo axes
 */
TEST_CASE("AC2: INPOS returns all servo axes", "[inpos_handler][hardware]")
{
    if (!ensure_test_ready()) {
        TEST_IGNORE_MESSAGE("Hardware not ready - skipping test");
    }

    ParsedCommand cmd;
    char response[128];

    make_inpos_command(&cmd, '\0');  /* No axis = read all */

    esp_err_t ret = handle_inpos(&cmd, response, sizeof(response));
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Response should start with "OK " */
    TEST_ASSERT_EQUAL_STRING_LEN("OK ", response, 3);

    /* Response should contain X:, Y:, Z:, A:, B: */
    TEST_ASSERT_NOT_NULL(strstr(response, "X:"));
    TEST_ASSERT_NOT_NULL(strstr(response, "Y:"));
    TEST_ASSERT_NOT_NULL(strstr(response, "Z:"));
    TEST_ASSERT_NOT_NULL(strstr(response, "A:"));
    TEST_ASSERT_NOT_NULL(strstr(response, "B:"));

    ESP_LOGI(TAG, "Response: %s", response);
}

/* ==========================================================================
 * AC3: INPOS X returns single axis
 * ========================================================================== */

/**
 * AC3: INPOS X returns single servo axis
 */
TEST_CASE("AC3: INPOS X returns single axis", "[inpos_handler][hardware]")
{
    if (!ensure_test_ready()) {
        TEST_IGNORE_MESSAGE("Hardware not ready - skipping test");
    }

    ParsedCommand cmd;
    char response[64];

    make_inpos_command(&cmd, 'X');

    esp_err_t ret = handle_inpos(&cmd, response, sizeof(response));
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Response should be "OK X <0|1>\r\n" */
    TEST_ASSERT_EQUAL_STRING_LEN("OK ", response, 3);
    TEST_ASSERT_NOT_NULL(strstr(response, "X "));

    ESP_LOGI(TAG, "Response: %s", response);
}

/**
 * AC3: INPOS Y returns single servo axis
 */
TEST_CASE("AC3: INPOS Y returns single axis", "[inpos_handler][hardware]")
{
    if (!ensure_test_ready()) {
        TEST_IGNORE_MESSAGE("Hardware not ready - skipping test");
    }

    ParsedCommand cmd;
    char response[64];

    make_inpos_command(&cmd, 'Y');

    esp_err_t ret = handle_inpos(&cmd, response, sizeof(response));
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    TEST_ASSERT_NOT_NULL(strstr(response, "Y "));
}

/* ==========================================================================
 * AC5: INPOS C returns N/A for stepper axis
 * ========================================================================== */

/**
 * AC5: INPOS C returns N/A for stepper
 */
TEST_CASE("AC5: INPOS C returns N/A", "[inpos_handler][hardware]")
{
    if (!ensure_test_ready()) {
        TEST_IGNORE_MESSAGE("Hardware not ready - skipping test");
    }

    ParsedCommand cmd;
    char response[64];

    make_inpos_command(&cmd, 'C');

    esp_err_t ret = handle_inpos(&cmd, response, sizeof(response));
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Response should contain "N/A" */
    TEST_ASSERT_NOT_NULL(strstr(response, "N/A"));

    ESP_LOGI(TAG, "Response: %s", response);
}

/**
 * AC5: INPOS D returns N/A for stepper
 */
TEST_CASE("AC5: INPOS D returns N/A", "[inpos_handler][hardware]")
{
    if (!ensure_test_ready()) {
        TEST_IGNORE_MESSAGE("Hardware not ready - skipping test");
    }

    ParsedCommand cmd;
    char response[64];

    make_inpos_command(&cmd, 'D');

    esp_err_t ret = handle_inpos(&cmd, response, sizeof(response));
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    TEST_ASSERT_NOT_NULL(strstr(response, "N/A"));
}

/**
 * AC5: INPOS E returns N/A for discrete axis
 */
TEST_CASE("AC5: INPOS E returns N/A", "[inpos_handler][hardware]")
{
    if (!ensure_test_ready()) {
        TEST_IGNORE_MESSAGE("Hardware not ready - skipping test");
    }

    ParsedCommand cmd;
    char response[64];

    make_inpos_command(&cmd, 'E');

    esp_err_t ret = handle_inpos(&cmd, response, sizeof(response));
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    TEST_ASSERT_NOT_NULL(strstr(response, "N/A"));
}

/* ==========================================================================
 * AC6: Invalid axis returns error
 * ========================================================================== */

/**
 * AC6: INPOS Q returns error for invalid axis
 */
TEST_CASE("AC6: INPOS invalid axis returns error", "[inpos_handler][hardware]")
{
    if (!ensure_test_ready()) {
        TEST_IGNORE_MESSAGE("Hardware not ready - skipping test");
    }

    ParsedCommand cmd;
    char response[64];

    make_inpos_command(&cmd, 'Q');  /* Invalid axis */

    esp_err_t ret = handle_inpos(&cmd, response, sizeof(response));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);

    /* Response should contain "ERROR" */
    TEST_ASSERT_NOT_NULL(strstr(response, "ERROR"));

    ESP_LOGI(TAG, "Response: %s", response);
}

/* ==========================================================================
 * Parameter Validation Tests
 * ========================================================================== */

/**
 * Handler returns error for NULL command
 */
TEST_CASE("handle_inpos returns error for null command", "[inpos_handler]")
{
    char response[64];

    esp_err_t ret = handle_inpos(NULL, response, sizeof(response));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

/**
 * Handler returns error for NULL response buffer
 */
TEST_CASE("handle_inpos returns error for null response", "[inpos_handler]")
{
    ParsedCommand cmd;
    make_inpos_command(&cmd, '\0');

    esp_err_t ret = handle_inpos(&cmd, NULL, 64);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

/**
 * Handler returns error for zero response length
 */
TEST_CASE("handle_inpos returns error for zero response length", "[inpos_handler]")
{
    ParsedCommand cmd;
    char response[64];
    make_inpos_command(&cmd, '\0');

    esp_err_t ret = handle_inpos(&cmd, response, 0);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}
