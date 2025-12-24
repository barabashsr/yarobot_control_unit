/**
 * @file test_width_handler.c
 * @brief Integration tests for WIDTH command handler
 * @author YaRobot Team
 * @date 2025
 *
 * Tests acceptance criteria from Story 4-9:
 *   AC4: WIDTH C returns last measurement or -1.000
 *   AC7: WIDTH X returns error for non-C axis
 *
 * @note Tests require command_executor and floating_switch initialized.
 */

#include "unity.h"
#include "width_handler.h"
#include "floating_switch.h"
#include "command_executor.h"
#include "command_parser.h"
#include "config_commands.h"
#include "config_axes.h"
#include "safety_monitor.h"

#include <string.h>
#include "esp_log.h"

static const char* TAG = "test_width";

/* ==========================================================================
 * Test Helpers
 * ========================================================================== */

/**
 * @brief Create parsed command for WIDTH
 */
static void make_width_command(ParsedCommand* cmd, char axis)
{
    memset(cmd, 0, sizeof(*cmd));
    strncpy(cmd->verb, CMD_WIDTH, sizeof(cmd->verb) - 1);
    cmd->axis = axis;
    cmd->param_count = 0;
    cmd->has_str_param = false;
}

/**
 * @brief Check if test prerequisites are met
 */
static bool ensure_test_ready(void)
{
    /* Safety monitor must be initialized for floating_switch to work */
    if (!safety_monitor_is_initialized()) {
        return false;
    }

    /* Initialize floating_switch if needed */
    if (!floating_switch_is_initialized()) {
        if (floating_switch_init() != ESP_OK) {
            return false;
        }
    }

    return true;
}

/* ==========================================================================
 * AC4: WIDTH C returns measurement or -1.000
 * ========================================================================== */

/**
 * AC4: WIDTH C returns -1.000 when no measurement
 */
TEST_CASE("AC4: WIDTH C returns -1.000 no measurement", "[width_handler][hardware]")
{
    if (!ensure_test_ready()) {
        TEST_IGNORE_MESSAGE("Hardware not ready - skipping test");
    }

    /* Clear any existing measurement */
    floating_switch_clear_measurement();

    ParsedCommand cmd;
    char response[64];

    make_width_command(&cmd, 'C');

    esp_err_t ret = handle_width(&cmd, response, sizeof(response));
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Response should be "OK C -1.000\r\n" */
    TEST_ASSERT_EQUAL_STRING_LEN("OK ", response, 3);
    TEST_ASSERT_NOT_NULL(strstr(response, "C "));
    TEST_ASSERT_NOT_NULL(strstr(response, "-1.000"));

    ESP_LOGI(TAG, "Response: %s", response);
}

/**
 * AC4: WIDTH C with lowercase returns result
 */
TEST_CASE("AC4: WIDTH c (lowercase) works", "[width_handler][hardware]")
{
    if (!ensure_test_ready()) {
        TEST_IGNORE_MESSAGE("Hardware not ready - skipping test");
    }

    ParsedCommand cmd;
    char response[64];

    make_width_command(&cmd, 'c');  /* Lowercase */

    esp_err_t ret = handle_width(&cmd, response, sizeof(response));
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Response should contain OK */
    TEST_ASSERT_EQUAL_STRING_LEN("OK ", response, 3);
}

/**
 * AC4: WIDTH (no axis) defaults to C axis
 */
TEST_CASE("AC4: WIDTH no axis defaults to C", "[width_handler][hardware]")
{
    if (!ensure_test_ready()) {
        TEST_IGNORE_MESSAGE("Hardware not ready - skipping test");
    }

    ParsedCommand cmd;
    char response[64];

    make_width_command(&cmd, '\0');  /* No axis specified */

    esp_err_t ret = handle_width(&cmd, response, sizeof(response));
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Response should contain OK and C */
    TEST_ASSERT_EQUAL_STRING_LEN("OK ", response, 3);
    TEST_ASSERT_NOT_NULL(strstr(response, "C "));

    ESP_LOGI(TAG, "Response: %s", response);
}

/* ==========================================================================
 * AC7: Invalid axis returns error
 * ========================================================================== */

/**
 * AC7: WIDTH X returns error for non-C axis
 */
TEST_CASE("AC7: WIDTH X returns error", "[width_handler][hardware]")
{
    if (!ensure_test_ready()) {
        TEST_IGNORE_MESSAGE("Hardware not ready - skipping test");
    }

    ParsedCommand cmd;
    char response[128];

    make_width_command(&cmd, 'X');

    esp_err_t ret = handle_width(&cmd, response, sizeof(response));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);

    /* Response should contain "ERROR" and the error code */
    TEST_ASSERT_NOT_NULL(strstr(response, "ERROR"));
    TEST_ASSERT_NOT_NULL(strstr(response, ERR_WIDTH_C_ONLY));

    ESP_LOGI(TAG, "Response: %s", response);
}

/**
 * AC7: WIDTH Y returns error for non-C axis
 */
TEST_CASE("AC7: WIDTH Y returns error", "[width_handler][hardware]")
{
    if (!ensure_test_ready()) {
        TEST_IGNORE_MESSAGE("Hardware not ready - skipping test");
    }

    ParsedCommand cmd;
    char response[128];

    make_width_command(&cmd, 'Y');

    esp_err_t ret = handle_width(&cmd, response, sizeof(response));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);

    TEST_ASSERT_NOT_NULL(strstr(response, "ERROR"));
}

/**
 * AC7: WIDTH Z returns error for non-C axis
 */
TEST_CASE("AC7: WIDTH Z returns error", "[width_handler][hardware]")
{
    if (!ensure_test_ready()) {
        TEST_IGNORE_MESSAGE("Hardware not ready - skipping test");
    }

    ParsedCommand cmd;
    char response[128];

    make_width_command(&cmd, 'Z');

    esp_err_t ret = handle_width(&cmd, response, sizeof(response));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);

    TEST_ASSERT_NOT_NULL(strstr(response, "ERROR"));
}

/**
 * AC7: WIDTH A returns error for non-C axis
 */
TEST_CASE("AC7: WIDTH A returns error", "[width_handler][hardware]")
{
    if (!ensure_test_ready()) {
        TEST_IGNORE_MESSAGE("Hardware not ready - skipping test");
    }

    ParsedCommand cmd;
    char response[128];

    make_width_command(&cmd, 'A');

    esp_err_t ret = handle_width(&cmd, response, sizeof(response));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);

    TEST_ASSERT_NOT_NULL(strstr(response, "ERROR"));
}

/**
 * AC7: WIDTH D returns error for non-C axis
 */
TEST_CASE("AC7: WIDTH D returns error", "[width_handler][hardware]")
{
    if (!ensure_test_ready()) {
        TEST_IGNORE_MESSAGE("Hardware not ready - skipping test");
    }

    ParsedCommand cmd;
    char response[128];

    make_width_command(&cmd, 'D');

    esp_err_t ret = handle_width(&cmd, response, sizeof(response));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);

    TEST_ASSERT_NOT_NULL(strstr(response, "ERROR"));
}

/**
 * AC6: Invalid axis (Q) returns error
 */
TEST_CASE("AC7: WIDTH Q (invalid) returns error", "[width_handler][hardware]")
{
    if (!ensure_test_ready()) {
        TEST_IGNORE_MESSAGE("Hardware not ready - skipping test");
    }

    ParsedCommand cmd;
    char response[128];

    make_width_command(&cmd, 'Q');  /* Invalid axis */

    esp_err_t ret = handle_width(&cmd, response, sizeof(response));
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
TEST_CASE("handle_width returns error for null command", "[width_handler]")
{
    char response[64];

    esp_err_t ret = handle_width(NULL, response, sizeof(response));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

/**
 * Handler returns error for NULL response buffer
 */
TEST_CASE("handle_width returns error for null response", "[width_handler]")
{
    ParsedCommand cmd;
    make_width_command(&cmd, 'C');

    esp_err_t ret = handle_width(&cmd, NULL, 64);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

/**
 * Handler returns error for zero response length
 */
TEST_CASE("handle_width returns error for zero response length", "[width_handler]")
{
    ParsedCommand cmd;
    char response[64];
    make_width_command(&cmd, 'C');

    esp_err_t ret = handle_width(&cmd, response, 0);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

/* ==========================================================================
 * Handler Registration Test
 * ========================================================================== */

/**
 * width_handler_register succeeds
 */
TEST_CASE("width_handler_register succeeds", "[width_handler]")
{
    /* Note: This may fail if already registered, which is fine */
    esp_err_t ret = width_handler_register();

    /* Either success or already registered (ESP_ERR_NO_MEM if table full) */
    TEST_ASSERT_TRUE(ret == ESP_OK || ret == ESP_ERR_INVALID_STATE);
}
